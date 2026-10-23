// servo_tilt.ino -- OBSLAM payload: tilt-servo controller
//
// Drives one GS2010MG 180-degree servo (50 Hz PWM) that tilts the LiDAR cradle,
// reads the cradle angle back from an AS5600 magnetic encoder on the tilt axle
// (I2C; optional, a board without one behaves exactly like the pre-encoder
// firmware), and talks to the core server over USB serial. On the ESP32-S3 it
// also carries the RPLIDAR's 1 Mbaud UART stream, unchanged and both ways,
// between UART1 and the DevKitC-1's second USB socket, so the Slamtec SDK on
// the Mac talks to the LiDAR as if it were plugged in directly (see the
// "lidar" section and firmware/README.md). The protocol is the contract in
// docs/api.md, section 2:
//
//   115200 8N1, '\n'-terminated ASCII lines ('\r' is accepted as a terminator
//   too, so `screen` works), exactly one reply line per command. Angles are
//   servo degrees 0..RANGE (RANGE = 180 or 270). <meas> is the encoder angle
//   on that same scale (aligned with `E zero`); it is omitted / `nan` on a
//   board without an encoder.
//
//   I                           -> OBSLAM-TILT v1
//   ?                           -> STATUS angle=<deg> meas=<deg|nan> min=<deg> max=<deg> range=<180|270>
//                                         us_min=<us> us_max=<us> enc=<ok|nomag|weak|strong|err|none> settled=<0|1>
//                                         lidar_rx=<bytes from lidar> lidar_tx=<bytes to lidar>
//                                         lidar_drop=<bytes dropped, host link full> lidar_ovf=<uart overflow events>
//                                         ip=<ip|none> tcp_rx=<bytes from 8899 client> tcp_tx=<bytes to 8899 client>
//   N                           -> NET ssid=<ssid|none> ip=<ip|none> rssi=<dBm|none> clients_lidar=<0|1> clients_tilt=<0|1>
//   N <ssid> [<password>]       -> OK         store the Wi-Fi network in NVS and (re)join it (S3 only;
//                                             plain ESP32: ERR no wifi). Unsolicited `NET ip=<ip>` on
//                                             Serial each time the link gets an address.
//
//                                         ... buf=<used>/<cap> audio=<on|off>
//   A                           -> AUDIO rate=48000 shift=<8..16> clients=<0|1> frames=<n> dropped=<n> i2s_timeouts=<n>
//   A shift <8..16>             -> OK         mic gain (right shift of the 32-bit slot), persisted
//                                             (plain ESP32: ERR no audio)
//
//   The same protocol runs on TCP port 8900 (S3, Wi-Fi up); each reply goes
//   back to the link its command came from. TCP 8899 is the raw LiDAR bridge,
//   TCP 8901 the four-microphone AUD1 stream (docs/api.md §5.2).
//   T <deg>                     -> OK <deg> [<meas>]   move now, clamped, reply once the pulse is set
//   S <deg> <ms>                -> OK <deg> [<meas>]   ease linearly over <ms>, reply when done
//   H                           -> OK <deg> [<meas>]   go to RANGE/2
//   W [ms]                      -> OK <meas> settled=<0|1>  wait until the encoder is stable
//                                  (|delta| < 0.15 deg over 150 ms) or <ms> elapses (default 1500,
//                                  max 10000). No encoder: fixed 300 ms wait, then OK <angle> settled=1
//   E                           -> ENC raw=<0..4095> deg=<deg> zero=<deg> dir=<1|-1> status=<ok|nomag|weak|strong|err|none>
//   E zero                      -> OK <meas>   align the encoder scale so meas == commanded angle, persisted
//   E dir                       -> OK <meas>   flip the encoder direction (re-aligned at this pose), persisted
//   L <min> <max>               -> OK         soft limits (deg), persisted in NVS
//   C <us_min> <us_max> <range> -> OK         pulse calibration, persisted in NVS
//   anything else               -> ERR <msg>
//
// Boards: ESP32-S3 DevKitC-1 (esp32:esp32:esp32s3:CDCOnBoot=cdc, the target:
// tilt protocol on the native USB socket, LiDAR bytes on the UART socket) and
// plain ESP32 DevKit (esp32:esp32:esp32, one USB socket, no pass-through).
// Compiles for both. See firmware/README.md.

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <Wire.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include <ESPmDNS.h>
#include <WiFi.h>
#include <errno.h>
#include <lwip/sockets.h>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <esp_rom_gpio.h>
#include <esp_timer.h>
#include <soc/gpio_sig_map.h>
#endif

// ---------------------------------------------------------------- settings --

// Pin map follows the hardware team's pin_connection.md (24 Oct 2026) on the
// ESP32-S3 N16R8 board: servo 17, LiDAR UART 13/14, I2C 11/12, mics on 4-7.
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define SERVO_PIN        17        // servo signal (pin_connection.md §6)
#else
#define SERVO_PIN        13        // plain ESP32 bench board
#endif
#define SERVO_HZ         50

// ESP32Servo defaults to a 10-bit timer (19.5 us per tick, ~1.8 deg on a 180
// servo). Widen it: 16 bits on the plain ESP32 (LEDC supports up to 20),
// 14 bits on the S2/S3/C3 family (their LEDC ceiling).
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32C3)
#define SERVO_TIMER_BITS 14
#else
#define SERVO_TIMER_BITS 16
#endif
#define SERIAL_BAUD      115200
#define BANNER           "OBSLAM-TILT v1"

#define EASE_STEP_MS     20        // one 50 Hz servo frame per easing step
#define EASE_MAX_MS      60000     // `S` durations above this are rejected
#define CMD_LINE_MAX     128       // longest accepted command line, incl. NUL (N needs ~100)

// Pulse bounds the ESP32Servo library itself enforces (MIN/MAX_PULSE_WIDTH).
// `C` refuses anything outside these.
#define US_ABS_MIN       500
#define US_ABS_MAX       2500

// Factory defaults from the contract: 500 2500 180, soft limits = full range.
// The GS2010MG is a 180-degree servo; its real pulse window may differ a bit,
// measure it and set it with `C`.
#define DEF_US_MIN       500
#define DEF_US_MAX       2500
#define DEF_RANGE        180

#define NVS_NAMESPACE    "tilt"

// AS5600 magnetic encoder on the tilt axle. The I2C pins are the board's
// default bus: GPIO 8/9 on the ESP32-S3 DevKitC-1, GPIO 21/22 on the plain
// ESP32 DevKit (8/9 are flash pins there).
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define AS5600_SDA       11        // shared with the IMU (0x68); AS5600 is 0x36
#define AS5600_SCL       12
#else
#define AS5600_SDA       21
#define AS5600_SCL       22
#endif
#define AS5600_ADDR      0x36
#define AS5600_I2C_HZ    400000
#define AS5600_I2C_TIMEOUT_MS 20   // a hung bus costs at most this per transaction

#define AS5600_REG_STATUS    0x0B  // MH bit 3, ML bit 4, MD bit 5
#define AS5600_REG_RAW_ANGLE 0x0C  // 0x0C bits 3:0 = angle 11:8, 0x0D = angle 7:0
#define AS5600_ST_MH         0x08  // magnet too strong (AGC at minimum gain)
#define AS5600_ST_ML         0x10  // magnet too weak (AGC at maximum gain)
#define AS5600_ST_MD         0x20  // magnet detected

#define ENC_POLL_MS      10        // ~100 Hz
#define ENC_ERR_POLL_MS  100       // back off while reads fail (cable pulled, bus hung)

// Settle detector: stable when the encoder stayed within SETTLE_TOL_DEG
// peak-to-peak for SETTLE_WIN_MS. SETTLE_N samples at ENC_POLL_MS span the
// window.
#define SETTLE_WIN_MS    150
#define SETTLE_TOL_DEG   0.15f
#define SETTLE_N         (SETTLE_WIN_MS / ENC_POLL_MS + 1)

#define WAIT_DEF_MS      1500      // `W` without an argument
#define WAIT_MAX_MS      10000
#define WAIT_NOENC_MS    300       // `W` on a board without an encoder: fixed wait

// LiDAR UART pass-through (ESP32-S3 only). The RPLIDAR (1 Mbaud TTL UART,
// 3.3 V logic) hangs off UART1 and its bytes are copied unchanged, both ways,
// to the Mac over the DevKitC-1's second USB socket. A plain ESP32 has one
// USB socket, so there the pass-through is not built.
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define LIDAR_PASSTHROUGH   1
#define LIDAR_RX_PIN        13      // LiDAR TX (yellow) -> this pin
#define LIDAR_TX_PIN        14      // LiDAR RX (green) <- this pin
#define LIDAR_BAUD          1000000
#define LIDAR_RX_BUF        16384   // UART1 RX ring, ~160 ms of scan data
#define LIDAR_TX_BUF        1024    // UART1 TX ring; SDK commands are a few bytes
#define LIDAR_HOST_RX_BUF   4096
#define LIDAR_HOST_TX_BUF   4096
#define LIDAR_CHUNK         512     // LiDAR -> Mac bytes per pump call
#define LIDAR_CMD_CHUNK     64      // Mac -> LiDAR bytes per pump call
#define LIDAR_RX_HIGH_WATER (LIDAR_RX_BUF - 2 * LIDAR_CHUNK)
#define LIDAR_RX_FIFO_FULL  64      // RX FIFO interrupt threshold: leaves 64 bytes
                                    // (640 us at 1 Mbaud) of ISR latency margin
#if ARDUINO_USB_CDC_ON_BOOT
// Build CDCOnBoot=cdc (the documented one): Serial is the native USB socket
// and carries the tilt protocol; Serial0 is UART0 on the CP2102N "UART"
// socket and carries the LiDAR bytes.
#define LIDAR_HOST          Serial0
#define LIDAR_HOST_DESC     "UART socket"
#elif ARDUINO_USB_MODE
// Build without CDCOnBoot: Serial *is* Serial0 (the same object, the UART
// socket), so the tilt protocol sits there and the LiDAR bytes go out of the
// native USB socket through our own HWCDC instance. Same invariant (one host
// link each), sockets swapped.
#define LIDAR_HOST          LidarUsb
#define LIDAR_HOST_IS_HWCDC 1
#define LIDAR_HOST_DESC     "native USB socket"
#else
#error "ESP32-S3: build with USBMode=hwcdc (the default) and CDCOnBoot=cdc"
#endif
#endif

// Wi-Fi (ESP32-S3 only): station mode with credentials from `N`, mDNS name
// obslam-tilt.local, and two TCP servers, one client each (a new connection
// replaces the old one):
//   8899  raw LiDAR bridge to UART1, same bytes as the USB pass-through
//   8900  the tilt line protocol, same parser and replies as on Serial
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define NET_ENABLED         1
#define NET_HOSTNAME        "obslam-tilt"
#define NET_LIDAR_PORT      8899
#define NET_TILT_PORT       8900
#define NET_POLL_MS         50      // accept / connection checks
#define NET_RETRY_MS        15000   // kick a reconnect if still down after this
#endif

// Four INMP441 microphones (ESP32-S3 only; docs/api.md §5, pin_connection.md §3),
// captured simultaneously at 48 kHz and streamed on TCP 8901.
//   I2S0 master RX: BCLK GPIO4, WS GPIO5 (drives all four mics), DIN GPIO7 = mic1 L / mic2 R
//   I2S1 slave  RX: DIN GPIO6 = mic3 L / mic4 R, clocked from the SAME GPIO4/5 pads.
// AUDIO_CLOCK_JUMPERS 0 (default): I2S1's BCK/WS inputs are routed from pads 4/5
// through the GPIO matrix, no wires. 1: the documented fallback, I2S1 takes its
// clocks from GPIO8/9, which then need the GPIO4->8 and GPIO5->9 jumpers. The
// hardware team says NOT to fit those jumpers, so leave this at 0.
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define AUDIO_ENABLED       1
#ifndef AUDIO_CLOCK_JUMPERS
#define AUDIO_CLOCK_JUMPERS 0
#endif
#define AUD_BCLK_PIN        4
#define AUD_WS_PIN          5
#define AUD_DIN0_PIN        7       // I2S0: mic1 (L/R=GND, left) + mic2 (L/R=3V3, right)
#define AUD_DIN1_PIN        6       // I2S1: mic3 (left) + mic4 (right)
#define AUD_SPARE0_PIN      8       // unconnected; kept a plain input (jumper target if JUMPERS=1)
#define AUD_SPARE1_PIN      9
#define AUD_RATE            48000
#define AUD_N               480     // samples per channel per frame = 10 ms
#define AUD_CH              4
#define AUD_PORT            8901
#define AUD_MAGIC           0x31445541u  // 'AUD1'
#define AUD_HDR_BYTES       28
#define AUD_FRAME_BYTES     (AUD_HDR_BYTES + AUD_N * AUD_CH * 2)  // 3868
#define AUD_RING_FRAMES     200     // 2 s in PSRAM (~774 KB)
#define AUD_RING_FRAMES_SRAM 16     // 160 ms if there is no PSRAM
#define AUD_SHIFT_DEF       14
#define AUD_SHIFT_MIN       8
#define AUD_SHIFT_MAX       16
#define AUD_DMA_DESC        12      // 12 x 240 frames = 60 ms of DMA cover per unit
#define AUD_DMA_FRAMES      240
#define AUD_READ_TIMEOUT_MS 100
#define AUD_TASK_CORE       0       // with Wi-Fi; the loop() stays alone on core 1
#define AUD_TASK_PRIO       10      // above idle work, below lwIP (18) and Wi-Fi (23)
#define AUD_SEND_FRAMES     2       // frames pushed per service() call at most
#endif

// ------------------------------------------------------------------- state --

struct Cal {
  int   us_min;   // pulse at 0 deg
  int   us_max;   // pulse at RANGE deg
  int   range;    // 180 or 270
  float lim_min;  // soft limit, servo degrees
  float lim_max;
};

enum EncStatus : uint8_t { ENC_NONE = 0, ENC_OK, ENC_NOMAG, ENC_WEAK, ENC_STRONG, ENC_ERR };
static const char *const ENC_STATUS_STR[] = {"none", "ok", "nomag", "weak", "strong", "err"};

struct Enc {
  bool      present;  // boot probe found the chip; false = never touch I2C again
  bool      valid;    // raw/meas hold at least one successful read
  EncStatus status;
  uint16_t  raw;      // last good RAW ANGLE, 0..4095
  float     meas;     // raw on the servo scale (zero and dir applied)
  float     zero;     // encoder degrees (0..360) at servo 0 deg, persisted
  int       dir;      // +1 or -1, persisted
};

static Servo       servo;
static Preferences prefs;
static Cal         cal;
static Enc         enc;
static float       cur_deg = 0.0f;  // last commanded angle; the servo itself has no feedback
static int         cur_us  = -1;    // last pulse written, to spot a real change

static unsigned long enc_last_poll = 0;

static float   settle_buf[SETTLE_N];
static uint8_t settle_head  = 0;
static uint8_t settle_count = 0;
static bool    settled      = false;

// One line assembler per host link; replies go to the link the line came from.
struct LineBuf {
  char   buf[CMD_LINE_MAX];
  size_t len;
  bool   overflow;
};
static LineBuf serial_line = {};
static Print  *out         = &Serial;  // reply link of the command being handled

#ifdef LIDAR_HOST_IS_HWCDC
static HWCDC LidarUsb;
#endif
// Pass-through counters, printed by `?` on every board (all zero without it).
// 32-bit: lidar_rx wraps after 4 GiB, about 12 h of continuous scanning.
static uint32_t          lidar_rx   = 0;  // bytes LiDAR -> Mac
static uint32_t          lidar_tx   = 0;  // bytes Mac -> LiDAR
static uint32_t          lidar_drop = 0;  // bytes discarded: host link full, RX ring at high water
static volatile uint32_t lidar_ovf  = 0;  // UART1 driver overflow events (FIFO or ring); bytes unknown
static uint32_t          tcp_rx     = 0;  // bytes the TCP 8899 client sent (-> LiDAR)
static uint32_t          tcp_tx     = 0;  // bytes sent to the TCP 8899 client (LiDAR ->)

#ifdef NET_ENABLED
static char          net_ssid[33] = "";
static char          net_pass[65] = "";
static bool          net_started   = false;
static bool          mdns_started  = false;
static volatile bool net_got_ip    = false;  // set by the Wi-Fi event task
static unsigned long net_down_since = 0;
static unsigned long net_last_poll  = 0;

static NetworkServer lidar_server(NET_LIDAR_PORT);
static NetworkServer tilt_server(NET_TILT_PORT);
static NetworkClient lidar_client;
static NetworkClient tilt_client;
static bool          lidar_client_on = false;
static bool          tilt_client_on  = false;
static LineBuf       tilt_line       = {};

// LiDAR -> TCP bytes already taken off UART1 but not yet accepted by the socket.
// LiDAR -> TCP goes through a large ring so Wi-Fi stalls delay the stream instead
// of losing it: ~100 KB/s of scan data, 1 MB in PSRAM = ~10 s of stall cover
// (the N16R8 has 8 MB). Falls back to 32 KB of internal RAM without PSRAM.
#define TCP_RING_PSRAM  (1024 * 1024)
#define TCP_RING_SRAM   (32 * 1024)
#define TCP_SEND_MAX    8192            // bytes per send() call
static uint8_t* tcp_ring = nullptr;
static size_t   tcp_ring_cap = 0;
static size_t   tcp_head = 0;           // next write
static size_t   tcp_tail = 0;           // next send
static size_t   tcp_pend_len = 0;       // bytes waiting in the ring
#endif

#ifdef AUDIO_ENABLED
struct __attribute__((packed)) AudHdr {
  uint32_t magic;
  uint32_t seq;
  uint64_t t_us;
  uint32_t sample_rate;
  uint16_t channels;
  uint16_t n;
  uint16_t shift;
  uint16_t flags;  // bit0 I2S read timeout in this frame, bit1 frames dropped before this one
};
static_assert(sizeof(AudHdr) == AUD_HDR_BYTES, "AUD1 header is 28 bytes");

static i2s_chan_handle_t aud_rx0 = nullptr;  // I2S0 master: mics 1, 2
static i2s_chan_handle_t aud_rx1 = nullptr;  // I2S1 slave:  mics 3, 4
static bool              aud_on  = false;    // both units running and the capture task up
static const char       *aud_clock_desc = "none";

// Frame ring, single producer (capture task, core 0) / single consumer
// (service(), core 1). aud_w and aud_r are free-running frame counters; each
// side writes only its own, with release/acquire ordering.
static uint8_t          *aud_ring        = nullptr;
static uint32_t          aud_ring_frames = 0;
static volatile uint32_t aud_w = 0;
static volatile uint32_t aud_r = 0;
static volatile bool     aud_streaming = false;  // a client is connected: producer fills the ring

static volatile uint8_t  aud_shift        = AUD_SHIFT_DEF;
static volatile uint32_t aud_frames       = 0;   // frames captured since boot
static volatile uint32_t aud_dropped      = 0;   // samples per channel dropped (ring full)
static volatile uint32_t aud_timeouts     = 0;   // I2S reads that timed out

static NetworkServer aud_server(AUD_PORT);
static NetworkClient aud_client;
static bool          aud_client_on = false;
static size_t        aud_send_off  = 0;          // bytes of the frame at aud_r already sent
#endif

// ----------------------------------------------------------------- helpers --

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float wrap360(float d) {
  d = fmodf(d, 360.0f);
  if (d < 0.0f) d += 360.0f;
  return d;
}

// Every target goes through here: [0, range] first, then the soft limits.
static float clampTarget(float deg) {
  deg = clampf(deg, 0.0f, (float)cal.range);
  deg = clampf(deg, cal.lim_min, cal.lim_max);
  return deg;
}

static int degToUs(float deg) {
  float us = (float)cal.us_min +
             deg * (float)(cal.us_max - cal.us_min) / (float)cal.range;
  long i = lroundf(us);
  if (i < cal.us_min) i = cal.us_min;
  if (i > cal.us_max) i = cal.us_max;
  return (int)i;
}

// ------------------------------------------------------------------ settle --

static void settleReset() {
  settle_head  = 0;
  settle_count = 0;
  settled      = false;
}

static void settlePush(float v) {
  settle_buf[settle_head] = v;
  settle_head = (uint8_t)((settle_head + 1) % SETTLE_N);
  if (settle_count < SETTLE_N) settle_count++;
  if (settle_count < SETTLE_N) {  // window not full yet
    settled = false;
    return;
  }
  float lo = v, hi = v;
  for (uint8_t i = 0; i < SETTLE_N; i++) {
    if (settle_buf[i] < lo) lo = settle_buf[i];
    if (settle_buf[i] > hi) hi = settle_buf[i];
  }
  settled = (hi - lo) < SETTLE_TOL_DEG;
}

// ----------------------------------------------------------------- encoder --

// Register read with a repeated start. False when the chip does not answer or
// returns short; the caller keeps the previous value in that case.
static bool as5600Read(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, n) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

// Encoder degrees (raw * 360/4096) onto the servo scale: direction and zero
// applied, then wrapped into the 360-degree window centred on range/2 so meas
// stays continuous just below 0 and just above RANGE (a cradle resting 1 deg
// past the end stop reads -1, not 359).
static float encMeasFromRaw(uint16_t raw) {
  float deg = (float)raw * (360.0f / 4096.0f);
  float m   = (float)enc.dir * (deg - enc.zero);
  float lo  = (float)cal.range / 2.0f - 180.0f;
  return lo + wrap360(m - lo);
}

// One sample: STATUS, then RAW ANGLE. Only called when enc.present.
static void encSample() {
  uint8_t st, ang[2];
  if (!as5600Read(AS5600_REG_STATUS, &st, 1) || !as5600Read(AS5600_REG_RAW_ANGLE, ang, 2)) {
    enc.status = ENC_ERR;  // raw/meas keep the last good value
    settleReset();
    return;
  }
  if (!(st & AS5600_ST_MD))   enc.status = ENC_NOMAG;
  else if (st & AS5600_ST_ML) enc.status = ENC_WEAK;
  else if (st & AS5600_ST_MH) enc.status = ENC_STRONG;
  else                        enc.status = ENC_OK;

  enc.raw   = (uint16_t)(((ang[0] & 0x0F) << 8) | ang[1]);
  enc.meas  = encMeasFromRaw(enc.raw);
  enc.valid = true;

  // Without a magnet the angle is noise: never call that "settled".
  if (enc.status == ENC_NOMAG) settleReset();
  else                         settlePush(enc.meas);
}

// Non-blocking ~100 Hz poll; call from every wait loop so STATUS and the
// settle detector stay fresh while a command blocks.
static void encPoll() {
  if (!enc.present) return;
  unsigned long now    = millis();
  unsigned long period = (enc.status == ENC_ERR) ? ENC_ERR_POLL_MS : ENC_POLL_MS;
  if (now - enc_last_poll < period) return;
  enc_last_poll = now;
  encSample();
}

// Probe at boot. Absent: enc=none for the rest of the run and the bus is
// released, so a board without an AS5600 behaves exactly as before.
static void encInit() {
  enc.present = false;
  enc.valid   = false;
  enc.status  = ENC_NONE;
  enc.raw     = 0;
  enc.meas    = 0.0f;
  settleReset();

  if (!Wire.begin(AS5600_SDA, AS5600_SCL, AS5600_I2C_HZ)) return;
  Wire.setTimeOut(AS5600_I2C_TIMEOUT_MS);

  uint8_t st;
  for (int i = 0; i < 3 && !enc.present; i++) {
    if (as5600Read(AS5600_REG_STATUS, &st, 1)) enc.present = true;
    else delay(10);
  }
  if (!enc.present) {
    Wire.end();
    return;
  }
  encSample();
  enc_last_poll = millis();
}

// Current encoder reading in encoder degrees (unzeroed), for E zero / E dir.
static float encRawDeg() { return (float)enc.raw * (360.0f / 4096.0f); }

// Re-align the encoder scale so meas == the commanded angle at this pose.
static void encAlignHere() {
  enc.zero = wrap360(encRawDeg() - (float)enc.dir * cur_deg);
  enc.meas = encMeasFromRaw(enc.raw);
  settleReset();  // the window holds values on the old scale
}

// ------------------------------------------------------------------- lidar --

#ifdef LIDAR_PASSTHROUGH

// Bytes a UART accepts right now without blocking. availableForWrite()
// reports the TX ring's free space, but falls back to the 128-byte FIFO's
// free space once the ring is full, so anything at or below the FIFO size
// counts as no room. The driver stores each write as ring items with their
// own headers, hence the slack.
static int uartRoom(int available_for_write) {
  const int slack = 32;
  return available_for_write > SOC_UART_FIFO_LEN + slack ? available_for_write - slack : 0;
}

static int hostRoom() {
#ifdef LIDAR_HOST_IS_HWCDC
  return LIDAR_HOST.availableForWrite();  // byte ring, zero timeout: never blocks
#else
  return uartRoom(LIDAR_HOST.availableForWrite());
#endif
}

static void lidarInit() {
  Serial1.setRxBufferSize(LIDAR_RX_BUF);
  Serial1.setTxBufferSize(LIDAR_TX_BUF);
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  Serial1.setRxFIFOFull(LIDAR_RX_FIFO_FULL);
  // Runs in the core's UART event task: count what the driver itself lost.
  Serial1.onReceiveError([](hardwareSerial_error_t e) {
    if (e == UART_FIFO_OVF_ERROR || e == UART_BUFFER_FULL_ERROR) lidar_ovf = lidar_ovf + 1;
  });

  LIDAR_HOST.setRxBufferSize(LIDAR_HOST_RX_BUF);
  LIDAR_HOST.setTxBufferSize(LIDAR_HOST_TX_BUF);
#ifdef LIDAR_HOST_IS_HWCDC
  LIDAR_HOST.setTxTimeoutMs(0);  // never wait for the Mac
  LIDAR_HOST.begin();
#else
  LIDAR_HOST.begin(LIDAR_BAUD, SERIAL_8N1);
#endif
}

#ifdef NET_ENABLED
static void tcpRingInit() {
  if (tcp_ring) return;
  if (psramFound()) {
    tcp_ring = (uint8_t*)ps_malloc(TCP_RING_PSRAM);
    if (tcp_ring) tcp_ring_cap = TCP_RING_PSRAM;
  }
  if (!tcp_ring) {
    tcp_ring = (uint8_t*)malloc(TCP_RING_SRAM);
    tcp_ring_cap = tcp_ring ? TCP_RING_SRAM : 0;
  }
}

static void lidarClientDrop() {
  lidar_client.stop();
  lidar_client_on = false;
  lidar_drop  += (uint32_t)tcp_pend_len;  // taken off the UART, never delivered
  tcp_pend_len = 0;
  tcp_head = tcp_tail = 0;
}

// LiDAR -> TCP client. Only new UART bytes are read once the previous chunk
// is fully in the socket; send() is MSG_DONTWAIT, so a slow Wi-Fi link never
// blocks the loop, it just leaves bytes in the UART ring.
static int lidarToTcp(int avail) {
  if (!tcp_ring_cap) return avail;
  // 1) UART -> ring: take everything that fits, in contiguous pieces.
  while (avail > 0 && tcp_pend_len < tcp_ring_cap) {
    size_t room = tcp_ring_cap - tcp_pend_len;
    size_t run  = tcp_ring_cap - tcp_head;          // contiguous space up to the wrap
    size_t n    = (size_t)avail;
    if (n > room) n = room;
    if (n > run)  n = run;
    n = Serial1.read(tcp_ring + tcp_head, n);
    if (n == 0) break;
    tcp_head = (tcp_head + n) % tcp_ring_cap;
    tcp_pend_len += n;
    lidar_rx += (uint32_t)n;
    avail -= (int)n;
  }
  // 2) ring -> socket: one non-blocking send of the contiguous run at the tail.
  if (tcp_pend_len > 0) {
    size_t run = tcp_ring_cap - tcp_tail;
    size_t n   = tcp_pend_len < run ? tcp_pend_len : run;
    if (n > TCP_SEND_MAX) n = TCP_SEND_MAX;
    int s = send(lidar_client.fd(), tcp_ring + tcp_tail, n, MSG_DONTWAIT);
    if (s > 0) {
      tcp_tail = (tcp_tail + (size_t)s) % tcp_ring_cap;
      tcp_pend_len -= (size_t)s;
      tcp_tx       += (uint32_t)s;
    } else if (s < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
      lidarClientDrop();  // peer gone; the USB pass-through takes over again
    }
  }
  return avail;  // > 0 only when the ring is full: the high-water discard below counts it
}
#endif

// One chunk per direction, never blocks: every read is zero-timeout and
// every write is capped by the receiver's free space. Called from loop() and
// from every wait loop, so it runs at a few kHz idle and at 1 kHz inside S/W.
// LiDAR -> host bytes have one consumer at a time: the TCP 8899 client when
// one is connected, the USB pass-through otherwise.
static void lidarPump() {
  static uint8_t buf[LIDAR_CHUNK];

  // LiDAR -> Mac: scan data, ~100 KB/s while scanning.
  int avail = Serial1.available();
  if (avail > 0) {
#ifdef NET_ENABLED
    if (lidar_client_on) {
      avail = lidarToTcp(avail);
    } else
#endif
    {
      int n = avail < LIDAR_CHUNK ? avail : LIDAR_CHUNK;
      int room = hostRoom();
      if (n > room) n = room;
      if (n > 0) {
        n = (int)Serial1.read(buf, (size_t)n);
        LIDAR_HOST.write(buf, (size_t)n);
        lidar_rx += (uint32_t)n;
        avail -= n;
      }
    }
    // Consumer stalled and the RX ring nearly full: discard the oldest
    // chunk here, counted, instead of letting the driver drop silently.
    if (avail > LIDAR_RX_HIGH_WATER) {
      lidar_drop += (uint32_t)Serial1.read(buf, LIDAR_CHUNK);
    }
  }
#ifdef NET_ENABLED
  else if (lidar_client_on && tcp_pend_len > 0) {
    lidarToTcp(0);  // finish the last chunk even when the LiDAR went quiet
  }
#endif

  // Mac -> LiDAR: the SDK's short commands, from the USB link and the TCP
  // client alike (only one of them is normally in use).
  avail = LIDAR_HOST.available();
  if (avail > 0) {
    int n = avail < LIDAR_CMD_CHUNK ? avail : LIDAR_CMD_CHUNK;
    int room = uartRoom(Serial1.availableForWrite());
    if (n > room) n = room;
    if (n > 0) {
      n = (int)LIDAR_HOST.read(buf, (size_t)n);
      Serial1.write(buf, (size_t)n);
      lidar_tx += (uint32_t)n;
    }
  }
#ifdef NET_ENABLED
  if (lidar_client_on) {
    avail = lidar_client.available();
    if (avail > 0) {
      int n = avail < LIDAR_CMD_CHUNK ? avail : LIDAR_CMD_CHUNK;
      int room = uartRoom(Serial1.availableForWrite());
      if (n > room) n = room;
      if (n > 0) {
        n = lidar_client.read(buf, (size_t)n);
        if (n > 0) {
          Serial1.write(buf, (size_t)n);
          lidar_tx += (uint32_t)n;
          tcp_rx   += (uint32_t)n;
        }
      }
    }
  }
#endif
}

#endif  // LIDAR_PASSTHROUGH

// ----------------------------------------------------------------- network --

#ifdef NET_ENABLED

static void fmtIp(char *buf, size_t n) {
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    snprintf(buf, n, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  } else {
    snprintf(buf, n, "none");
  }
}

// Start (first call) or re-point (after `N`) the station. Non-blocking apart
// from the driver's own disconnect of a live link (<= 1 s, only on `N`).
// The board always runs its own access point (the hardware team's SLAM-Test,
// 192.168.4.1) so a laptop can reach it in the field with no router; a network
// stored with `N` is joined as well.
#define AP_SSID   "SLAM-Test"
#define AP_PASS   "SLAMtest24"

static void netStart() {
  if (!net_started) {
    WiFi.persistent(false);  // credentials live in our NVS namespace only
    WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t) { net_got_ip = true; },
                 ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.setHostname(NET_HOSTNAME);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);
    WiFi.setSleep(false);    // modem sleep adds 100+ ms latency spikes to the stream
    WiFi.setAutoReconnect(true);
    lidar_server.setNoDelay(true);
    tilt_server.setNoDelay(true);
    lidar_server.begin();
    tilt_server.begin();
    net_started = true;
  }
  if (!net_ssid[0]) return;  // access point only
  WiFi.begin(net_ssid, net_pass[0] ? net_pass : nullptr);
  net_down_since = millis();
}

static void loadNet() {
  prefs.begin(NVS_NAMESPACE, true);
  prefs.getString("wifi_ssid", net_ssid, sizeof(net_ssid));
  prefs.getString("wifi_pass", net_pass, sizeof(net_pass));
  prefs.end();
}

static void saveNet() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putString("wifi_ssid", net_ssid);
  prefs.putString("wifi_pass", net_pass);
  prefs.end();
}

// Accepts and connection checks for the LiDAR bridge. Runs from service(),
// so the bridge also follows clients while S or W block.
static void netService() {
  if (!net_started) return;
  unsigned long now = millis();
  if (now - net_last_poll < NET_POLL_MS) return;
  net_last_poll = now;

  if (lidar_server.hasClient()) {
    NetworkClient c = lidar_server.accept();
    if (c) {
      if (lidar_client_on) lidarClientDrop();  // a new connection replaces the old one
      lidar_client    = c;
      lidar_client_on = true;
    }
  }
  if (lidar_client_on && !lidar_client.connected()) lidarClientDrop();
}

// Wi-Fi housekeeping and the TCP tilt link. Runs from loop() only, between
// commands, so the NET line never lands inside another command's reply and a
// replaced tilt client never receives a half-handled command's reply.
static void netLoop() {
  if (!net_started) return;

  if (net_got_ip) {
    net_got_ip = false;
    char ip[16];
    fmtIp(ip, sizeof(ip));
    Serial.printf("NET ip=%s\n", ip);
    if (!mdns_started && MDNS.begin(NET_HOSTNAME)) mdns_started = true;
  }

  // The driver's auto-reconnect covers ordinary drops; this covers the rest
  // (AP gone for a long time, auth failure that stopped retrying).
  if (!net_ssid[0] || WiFi.status() == WL_CONNECTED) {
    net_down_since = millis();
  } else if (millis() - net_down_since > NET_RETRY_MS) {
    WiFi.reconnect();
    net_down_since = millis();
  }

  static unsigned long last = 0;
  if (millis() - last >= NET_POLL_MS) {
    last = millis();
    if (tilt_server.hasClient()) {
      NetworkClient c = tilt_server.accept();
      if (c) {
        if (tilt_client_on) tilt_client.stop();
        tilt_client    = c;
        tilt_client_on = true;
        tilt_line.len      = 0;
        tilt_line.overflow = false;
      }
    }
    if (tilt_client_on && !tilt_client.connected()) {
      tilt_client.stop();
      tilt_client_on = false;
    }
  }
}

#endif  // NET_ENABLED

// ------------------------------------------------------------------- audio --

#ifdef AUDIO_ENABLED

// Read exactly one 10 ms block (480 stereo frames of 32-bit slots) from a unit.
// False on timeout; the unread rest is zeroed so the frame stays well-formed.
static bool audRead(i2s_chan_handle_t ch, int32_t *buf) {
  const size_t want = AUD_N * 2 * sizeof(int32_t);
  size_t got = 0;
  while (got < want) {
    size_t n = 0;
    esp_err_t e = i2s_channel_read(ch, (uint8_t *)buf + got, want - got, &n, AUD_READ_TIMEOUT_MS);
    got += n;
    if (e != ESP_OK) {
      memset((uint8_t *)buf + got, 0, want - got);
      return false;
    }
  }
  return true;
}

// Slave first, so it is already waiting when the master's first WS edge comes:
// both units then start on the same frame.
static void audStart() {
  i2s_channel_enable(aud_rx1);
  i2s_channel_enable(aud_rx0);
}

// After a timeout one unit may hold a partial block and the two would stay
// offset; a restart puts them back on a common first frame.
static void audRestart() {
  i2s_channel_disable(aud_rx0);
  i2s_channel_disable(aud_rx1);
  audStart();
}

static inline int16_t audSat(int32_t v, uint8_t shift) {
  v >>= shift;  // arithmetic: INMP441 data is signed, 24 bits left-justified in the 32-bit slot
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

// Capture task (core 0). Blocks only in i2s_channel_read, which sleeps on the
// DMA queue, so it costs the core almost nothing and never touches loop().
static void audTask(void *) {
  static int32_t raw0[AUD_N * 2];  // mic1, mic2 interleaved
  static int32_t raw1[AUD_N * 2];  // mic3, mic4 interleaved
  uint32_t seq          = 0;
  bool     drop_pending = false;
  // Timestamps: a sample-counted timeline (10 000 us per frame) anchored to
  // esp_timer. A block's read returns at or after its last sample arrived, so
  // now - (k+1)*10 ms is an upper bound on the timeline's start; the minimum of
  // those over a 1 s window removes the scheduling/DMA jitter, and moving to
  // each new window's minimum follows any slow rate offset.
  int64_t  anchor  = INT64_MAX, win_min = INT64_MAX;
  uint32_t k = 0, win = 0;

  for (;;) {
    uint16_t flags = 0;
    bool ok0 = audRead(aud_rx0, raw0);
    bool ok1 = audRead(aud_rx1, raw1);
    int64_t now = esp_timer_get_time();
    if (!ok0 || !ok1) {
      flags |= 1;
      aud_timeouts = aud_timeouts + 1;
    }

    int64_t est = now - (int64_t)(k + 1) * 10000;
    if (est < anchor)  anchor  = est;
    if (est < win_min) win_min = est;
    if (++win >= 100) {
      anchor  = win_min;
      win_min = INT64_MAX;
      win     = 0;
    }
    uint64_t t_us = (uint64_t)(anchor + (int64_t)k * 10000);
    k++;

    if (__atomic_load_n(&aud_streaming, __ATOMIC_ACQUIRE)) {
      uint32_t w = aud_w;
      uint32_t r = __atomic_load_n(&aud_r, __ATOMIC_ACQUIRE);
      if (w - r >= aud_ring_frames) {
        aud_dropped  = aud_dropped + AUD_N;  // client too slow: this frame is lost
        drop_pending = true;
      } else {
        uint8_t *slot = aud_ring + (size_t)(w % aud_ring_frames) * AUD_FRAME_BYTES;
        uint8_t  sh   = aud_shift;
        AudHdr   h;
        h.magic       = AUD_MAGIC;
        h.seq         = seq;
        h.t_us        = t_us;
        h.sample_rate = AUD_RATE;
        h.channels    = AUD_CH;
        h.n           = AUD_N;
        h.shift       = sh;
        h.flags       = (uint16_t)(flags | (drop_pending ? 2 : 0));
        memcpy(slot, &h, sizeof(h));
        int16_t *pcm = (int16_t *)(slot + AUD_HDR_BYTES);
        for (int i = 0; i < AUD_N; i++) {
          pcm[4 * i + 0] = audSat(raw0[2 * i + 0], sh);  // mic1
          pcm[4 * i + 1] = audSat(raw0[2 * i + 1], sh);  // mic2
          pcm[4 * i + 2] = audSat(raw1[2 * i + 0], sh);  // mic3
          pcm[4 * i + 3] = audSat(raw1[2 * i + 1], sh);  // mic4
        }
        drop_pending = false;
        __atomic_store_n(&aud_w, w + 1, __ATOMIC_RELEASE);
      }
    } else {
      drop_pending = false;  // nobody listening: nothing is "lost"
    }
    seq++;
    aud_frames = aud_frames + 1;

    if (flags & 1) {
      audRestart();
      anchor = win_min = INT64_MAX;  // new common start: new timeline
      k = win = 0;
    }
  }
}

static esp_err_t audNewUnits() {
  i2s_chan_config_t c0 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  c0.dma_desc_num  = AUD_DMA_DESC;
  c0.dma_frame_num = AUD_DMA_FRAMES;
  i2s_chan_config_t c1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
  c1.dma_desc_num  = AUD_DMA_DESC;
  c1.dma_frame_num = AUD_DMA_FRAMES;
  esp_err_t e = i2s_new_channel(&c0, nullptr, &aud_rx0);
  if (e == ESP_OK) e = i2s_new_channel(&c1, nullptr, &aud_rx1);
  return e;
}

static i2s_std_config_t audStdConfig(int bclk, int ws, int din) {
  i2s_std_config_t cfg = {};
  cfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUD_RATE);
  cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  cfg.gpio_cfg.bclk = (gpio_num_t)bclk;
  cfg.gpio_cfg.ws   = (gpio_num_t)ws;
  cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
  cfg.gpio_cfg.din  = (gpio_num_t)din;
  return cfg;
}

// Bring up both units and the capture task. False (audio=off) on any failure;
// the rest of the firmware is unaffected.
static bool audInit() {
  prefs.begin(NVS_NAMESPACE, true);
  uint8_t sh = prefs.getUChar("aud_shift", AUD_SHIFT_DEF);
  prefs.end();
  aud_shift = (sh >= AUD_SHIFT_MIN && sh <= AUD_SHIFT_MAX) ? sh : AUD_SHIFT_DEF;

  if (psramFound()) {
    aud_ring = (uint8_t *)ps_malloc((size_t)AUD_RING_FRAMES * AUD_FRAME_BYTES);
    if (aud_ring) aud_ring_frames = AUD_RING_FRAMES;
  }
  if (!aud_ring) {
    aud_ring = (uint8_t *)malloc((size_t)AUD_RING_FRAMES_SRAM * AUD_FRAME_BYTES);
    if (aud_ring) aud_ring_frames = AUD_RING_FRAMES_SRAM;
  }
  if (!aud_ring) return false;

  if (audNewUnits() != ESP_OK) return false;

  // I2S1 (slave) first: its init must not touch pads 4/5, which I2S0 owns.
#if AUDIO_CLOCK_JUMPERS
  i2s_std_config_t s1 = audStdConfig(AUD_SPARE0_PIN, AUD_SPARE1_PIN, AUD_DIN1_PIN);
  if (i2s_channel_init_std_mode(aud_rx1, &s1) != ESP_OK) return false;
  aud_clock_desc = "jumpers GPIO4->8, GPIO5->9";
#else
  i2s_std_config_t s1 = audStdConfig(I2S_GPIO_UNUSED, I2S_GPIO_UNUSED, AUD_DIN1_PIN);
  if (i2s_channel_init_std_mode(aud_rx1, &s1) != ESP_OK) {
    // A driver that insists on clock pins gets the unconnected GPIO8/9: as
    // slave clocks they are only inputs, and the matrix routing below
    // replaces them as I2S1's clock source anyway.
    s1 = audStdConfig(AUD_SPARE0_PIN, AUD_SPARE1_PIN, AUD_DIN1_PIN);
    if (i2s_channel_init_std_mode(aud_rx1, &s1) != ESP_OK) return false;
  }
  aud_clock_desc = "GPIO matrix from pads 4/5";
#endif

  i2s_std_config_t s0 = audStdConfig(AUD_BCLK_PIN, AUD_WS_PIN, AUD_DIN0_PIN);
  if (i2s_channel_init_std_mode(aud_rx0, &s0) != ESP_OK) return false;

#if !AUDIO_CLOCK_JUMPERS
  // I2S0 has made pads 4/5 outputs (I2S0I_BCK_OUT / I2S0I_WS_OUT). Turn their
  // input buffers on too (output stays enabled and routed), then feed the
  // same pads into I2S1's slave clock inputs. Each matrix input has a single
  // source register, so this is the last word on where I2S1 is clocked from.
  gpio_input_enable((gpio_num_t)AUD_BCLK_PIN);
  gpio_input_enable((gpio_num_t)AUD_WS_PIN);
  esp_rom_gpio_connect_in_signal(AUD_BCLK_PIN, I2S1I_BCK_IN_IDX, false);
  esp_rom_gpio_connect_in_signal(AUD_WS_PIN, I2S1I_WS_IN_IDX, false);
#endif

  audStart();
  if (xTaskCreatePinnedToCore(audTask, "audio", 4096, nullptr, AUD_TASK_PRIO, nullptr,
                              AUD_TASK_CORE) != pdPASS) {
    return false;
  }
  return true;
}

static void audClientDrop() {
  __atomic_store_n(&aud_streaming, false, __ATOMIC_RELEASE);
  aud_client.stop();
  aud_client_on = false;
  aud_send_off  = 0;
}

// Accepts, connection checks and the ring -> socket copy. Runs from service(),
// so the stream keeps flowing while S or W block. Non-blocking throughout.
static void audService() {
  if (!aud_on || !net_started) return;

  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= NET_POLL_MS) {
    last = now;
    if (aud_server.hasClient()) {
      NetworkClient c = aud_server.accept();
      if (c) {
        if (aud_client_on) audClientDrop();  // a new connection replaces the old one
        aud_client    = c;
        aud_client_on = true;
        aud_send_off  = 0;
        // Start the new client on fresh audio: skip whatever is queued.
        __atomic_store_n(&aud_r, __atomic_load_n(&aud_w, __ATOMIC_ACQUIRE), __ATOMIC_RELEASE);
        __atomic_store_n(&aud_streaming, true, __ATOMIC_RELEASE);
      }
    }
    if (aud_client_on && !aud_client.connected()) audClientDrop();
  }
  if (!aud_client_on) return;

  for (int i = 0; i < AUD_SEND_FRAMES; i++) {
    uint32_t r = aud_r;
    if (r == __atomic_load_n(&aud_w, __ATOMIC_ACQUIRE)) return;  // nothing queued
    const uint8_t *slot = aud_ring + (size_t)(r % aud_ring_frames) * AUD_FRAME_BYTES;
    int s = send(aud_client.fd(), slot + aud_send_off, AUD_FRAME_BYTES - aud_send_off, MSG_DONTWAIT);
    if (s > 0) {
      aud_send_off += (size_t)s;
      if (aud_send_off < AUD_FRAME_BYTES) return;  // socket full mid-frame: resume next call
      aud_send_off = 0;
      __atomic_store_n(&aud_r, r + 1, __ATOMIC_RELEASE);
    } else {
      if (s < 0 && errno != EWOULDBLOCK && errno != EAGAIN) audClientDrop();
      return;
    }
  }
}

#endif  // AUDIO_ENABLED

// Everything that must keep running while a command blocks.
static void service() {
  encPoll();
#ifdef NET_ENABLED
  netService();
#endif
#ifdef LIDAR_PASSTHROUGH
  lidarPump();
#endif
#ifdef AUDIO_ENABLED
  audService();
#endif
}

// ------------------------------------------------------------------- servo --

// deg must already be clamped.
static void applyAngle(float deg) {
  int us = degToUs(deg);
  if (us != cur_us) settleReset();  // a new pulse: the cradle is about to move
  servo.writeMicroseconds(us);
  cur_us  = us;
  cur_deg = deg;
}

// Wait ms while keeping the encoder polled and the LiDAR bytes moving.
static void waitPolling(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    service();
    delay(1);
  }
}

// Linear ease from the current angle to target over ms (already validated).
static void easeTo(float target, float ms) {
  float start = cur_deg;
  int   n     = (int)(ms / EASE_STEP_MS);
  if (n < 1) n = 1;
  unsigned long step_ms = (unsigned long)(ms / (float)n);
  for (int i = 1; i <= n; i++) {
    applyAngle(start + (target - start) * ((float)i / (float)n));
    waitPolling(step_ms);
  }
  applyAngle(target);  // land exactly, regardless of float rounding
}

// ----------------------------------------------------------------- replies --

// "135", "10.25", "0", "-1.5" -- up to two decimals, trailing zeros trimmed.
static void fmtDeg(char *out, size_t n, float deg) {
  snprintf(out, n, "%.2f", deg);
  char *dot = strchr(out, '.');
  if (!dot) return;
  char *end = out + strlen(out) - 1;
  while (end > dot && *end == '0') *end-- = '\0';
  if (*end == '.') *end = '\0';
  if (strcmp(out, "-0") == 0) strcpy(out, "0");
}

// meas, or "nan" when there is nothing to measure.
static void fmtMeas(char *out, size_t n) {
  if (enc.present && enc.valid) fmtDeg(out, n, enc.meas);
  else                          snprintf(out, n, "nan");
}

// "OK <deg>", plus " <meas>" when an encoder is fitted. Old parsers read only
// the second token, so the extra one is harmless.
static void replyOkDeg(float deg) {
  char a[16], m[16];
  fmtDeg(a, sizeof(a), deg);
  if (enc.present) {
    fmtMeas(m, sizeof(m));
    out->printf("OK %s %s\n", a, m);
  } else {
    out->printf("OK %s\n", a);
  }
}

static void replyStatus() {
  char a[16], m[16], lo[16], hi[16];
  fmtDeg(a,  sizeof(a),  cur_deg);
  fmtMeas(m, sizeof(m));
  fmtDeg(lo, sizeof(lo), cal.lim_min);
  fmtDeg(hi, sizeof(hi), cal.lim_max);
  char ip[16];
  unsigned long buf_used = 0, buf_cap = 0;
#ifdef NET_ENABLED
  fmtIp(ip, sizeof(ip));
  buf_used = (unsigned long)tcp_pend_len;
  buf_cap  = (unsigned long)tcp_ring_cap;
#else
  snprintf(ip, sizeof(ip), "none");
#endif
  bool audio = false;
#ifdef AUDIO_ENABLED
  audio = aud_on;
#endif
  out->printf("STATUS angle=%s meas=%s min=%s max=%s range=%d us_min=%d us_max=%d enc=%s settled=%d"
                " lidar_rx=%lu lidar_tx=%lu lidar_drop=%lu lidar_ovf=%lu ip=%s tcp_rx=%lu tcp_tx=%lu buf=%lu/%lu"
                " audio=%s\n",
                a, m, lo, hi, cal.range, cal.us_min, cal.us_max,
                ENC_STATUS_STR[enc.status], (enc.present && settled) ? 1 : 0,
                (unsigned long)lidar_rx, (unsigned long)lidar_tx,
                (unsigned long)lidar_drop, (unsigned long)lidar_ovf,
                ip, (unsigned long)tcp_rx, (unsigned long)tcp_tx,
                buf_used, buf_cap, audio ? "on" : "off");
}

// Magnet-spacing diagnostics. raw and deg are the chip's unzeroed reading;
// zero is on that same scale, so meas = dir * (deg - zero), wrapped.
static void replyEnc() {
  char d[16], z[16];
  if (enc.present && enc.valid) fmtDeg(d, sizeof(d), encRawDeg());
  else                          snprintf(d, sizeof(d), "nan");
  fmtDeg(z, sizeof(z), enc.zero);
  out->printf("ENC raw=%u deg=%s zero=%s dir=%d status=%s\n",
                (unsigned)enc.raw, d, z, enc.dir, ENC_STATUS_STR[enc.status]);
}

// --------------------------------------------------------------------- NVS --

// Bring whatever came out of NVS back into a sane, self-consistent state.
static void sanitizeCal() {
  if (cal.range != 180 && cal.range != 270) cal.range = DEF_RANGE;
  if (cal.us_min < US_ABS_MIN || cal.us_max > US_ABS_MAX || cal.us_min >= cal.us_max) {
    cal.us_min = DEF_US_MIN;
    cal.us_max = DEF_US_MAX;
  }
  if (!__builtin_isfinite(cal.lim_min) || !__builtin_isfinite(cal.lim_max)) {
    cal.lim_min = 0.0f;
    cal.lim_max = (float)cal.range;
  }
  cal.lim_min = clampf(cal.lim_min, 0.0f, (float)cal.range);
  cal.lim_max = clampf(cal.lim_max, 0.0f, (float)cal.range);
  if (cal.lim_min > cal.lim_max) {
    cal.lim_min = 0.0f;
    cal.lim_max = (float)cal.range;
  }
}

static void sanitizeEnc() {
  if (enc.dir != 1 && enc.dir != -1) enc.dir = 1;
  if (!__builtin_isfinite(enc.zero)) enc.zero = 0.0f;
  enc.zero = wrap360(enc.zero);
}

static void loadCal() {
  prefs.begin(NVS_NAMESPACE, false);
  cal.us_min  = prefs.getInt("us_min", DEF_US_MIN);
  cal.us_max  = prefs.getInt("us_max", DEF_US_MAX);
  cal.range   = prefs.getInt("range", DEF_RANGE);
  // Until the installed 13 kg servo is identified, the hardware team allows only
  // 1400-1600 us. With 500-2500 us over 180 deg that is 81-99 deg; `L` widens it.
  cal.lim_min = prefs.getFloat("lim_min", 81.0f);
  cal.lim_max = prefs.getFloat("lim_max", 99.0f);
  enc.zero    = prefs.getFloat("enc_zero", 0.0f);
  enc.dir     = prefs.getInt("enc_dir", 1);
  prefs.end();
  sanitizeCal();
  sanitizeEnc();
}

static void saveCal() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putInt("us_min", cal.us_min);
  prefs.putInt("us_max", cal.us_max);
  prefs.putInt("range", cal.range);
  prefs.putFloat("lim_min", cal.lim_min);
  prefs.putFloat("lim_max", cal.lim_max);
  prefs.end();
}

static void saveEnc() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putFloat("enc_zero", enc.zero);
  prefs.putInt("enc_dir", enc.dir);
  prefs.end();
}

// ----------------------------------------------------------------- parsing --

// Split s on spaces/tabs in place. Returns the token count (0 for a blank line).
static int tokenize(char *s, char **argv, int max_argv) {
  int argc = 0;
  while (*s && argc < max_argv) {
    while (*s == ' ' || *s == '\t') s++;
    if (!*s) break;
    argv[argc++] = s;
    while (*s && *s != ' ' && *s != '\t') s++;
    if (*s) *s++ = '\0';
  }
  // Anything left over means too many tokens; count it so the caller rejects it.
  while (*s == ' ' || *s == '\t') s++;
  if (*s) argc++;
  return argc;
}

// Whole-token finite float, else false.
static bool parseF(const char *s, float *out) {
  char  *end = nullptr;
  double v   = strtod(s, &end);
  if (end == s || *end != '\0' || !__builtin_isfinite(v)) return false;
  *out = (float)v;
  return true;
}

static void replyBadArgs() { out->println("ERR bad args"); }

// E zero / E dir need a reading with a magnet behind it.
static bool encUsable() {
  if (!enc.present) {
    out->println("ERR no encoder");
    return false;
  }
  if (!enc.valid || enc.status == ENC_ERR || enc.status == ENC_NOMAG) {
    out->printf("ERR enc=%s\n", ENC_STATUS_STR[enc.status]);
    return false;
  }
  return true;
}

static void handleLine(char *s) {
  char *argv[6];
  int   argc = tokenize(s, argv, 6);
  if (argc == 0) return;  // blank line: no reply
  if (argc > 4 || strlen(argv[0]) != 1) {
    out->println(argc > 4 ? "ERR bad args" : "ERR unknown cmd");
    return;
  }

  char cmd   = (char)toupper((unsigned char)argv[0][0]);
  int  nargs = argc - 1;
  float a = 0, b = 0, c = 0;

  switch (cmd) {
    case 'I':
      if (nargs != 0) { replyBadArgs(); return; }
      out->println(BANNER);
      return;

    case '?':
      if (nargs != 0) { replyBadArgs(); return; }
      replyStatus();
      return;

    case 'T':
      if (nargs != 1 || !parseF(argv[1], &a)) { replyBadArgs(); return; }
      applyAngle(clampTarget(a));
      replyOkDeg(cur_deg);
      return;

    case 'S':
      if (nargs != 2 || !parseF(argv[1], &a) || !parseF(argv[2], &b) ||
          b < 0.0f || b > (float)EASE_MAX_MS) {
        replyBadArgs();
        return;
      }
      easeTo(clampTarget(a), b);
      replyOkDeg(cur_deg);
      return;

    case 'H':
      if (nargs != 0) { replyBadArgs(); return; }
      applyAngle(clampTarget((float)cal.range / 2.0f));
      replyOkDeg(cur_deg);
      return;

    case 'W': {
      float ms = (float)WAIT_DEF_MS;
      if (nargs > 1 || (nargs == 1 && (!parseF(argv[1], &ms) || ms < 0.0f || ms > (float)WAIT_MAX_MS))) {
        replyBadArgs();
        return;
      }
      char m[16];
      if (!enc.present) {
        // Nothing to measure: a fixed settle delay so encoder-less callers
        // can still use W as their "wait for the servo" primitive.
        waitPolling(WAIT_NOENC_MS);
        fmtDeg(m, sizeof(m), cur_deg);
        out->printf("OK %s settled=1\n", m);
        return;
      }
      unsigned long budget = (unsigned long)ms;
      unsigned long t0     = millis();
      while (!settled && millis() - t0 < budget) {
        service();
        delay(1);
      }
      fmtMeas(m, sizeof(m));
      out->printf("OK %s settled=%d\n", m, settled ? 1 : 0);
      return;
    }

    case 'E': {
      if (nargs == 0) {
        replyEnc();
        return;
      }
      if (nargs != 1) { replyBadArgs(); return; }
      if (strcasecmp(argv[1], "zero") == 0) {
        if (!encUsable()) return;
        encAlignHere();
      } else if (strcasecmp(argv[1], "dir") == 0) {
        if (!encUsable()) return;
        enc.dir = -enc.dir;
        encAlignHere();  // keep meas == angle at this pose under the new direction
      } else {
        replyBadArgs();
        return;
      }
      saveEnc();
      char m[16];
      fmtMeas(m, sizeof(m));
      out->print("OK ");
      out->println(m);
      return;
    }

    case 'L':
      if (nargs != 2 || !parseF(argv[1], &a) || !parseF(argv[2], &b) ||
          a < 0.0f || b > (float)cal.range || a > b) {
        replyBadArgs();
        return;
      }
      cal.lim_min = a;
      cal.lim_max = b;
      saveCal();
      // The servo is left where it is; the next move command clamps.
      out->println("OK");
      return;

    case 'C': {
      if (nargs != 3 || !parseF(argv[1], &a) || !parseF(argv[2], &b) || !parseF(argv[3], &c)) {
        replyBadArgs();
        return;
      }
      long us_min = lroundf(a), us_max = lroundf(b), range = lroundf(c);
      if (us_min < US_ABS_MIN || us_max > US_ABS_MAX || us_min >= us_max ||
          (range != 180 && range != 270)) {
        replyBadArgs();
        return;
      }
      cal.us_min = (int)us_min;
      cal.us_max = (int)us_max;
      cal.range  = (int)range;
      sanitizeCal();  // pulls the soft limits into [0, range] if range shrank
      saveCal();
      // Same angle, new mapping: re-issue the pulse so the state stays truthful.
      applyAngle(clampTarget(cur_deg));
      // The meas wrap window follows range/2; refresh the cached value.
      if (enc.valid) enc.meas = encMeasFromRaw(enc.raw);
      out->println("OK");
      return;
    }

    case 'N': {
#ifdef NET_ENABLED
      if (nargs == 0) {
        char ip[16], rssi[8];
        fmtIp(ip, sizeof(ip));
        if (WiFi.status() == WL_CONNECTED) snprintf(rssi, sizeof(rssi), "%d", (int)WiFi.RSSI());
        else                               snprintf(rssi, sizeof(rssi), "none");
        out->printf("NET ssid=%s ip=%s rssi=%s clients_lidar=%d clients_tilt=%d\n",
                    net_ssid[0] ? net_ssid : "none", ip, rssi,
                    lidar_client_on ? 1 : 0, tilt_client_on ? 1 : 0);
        return;
      }
      // N <ssid> [<password>]: no password = open network. WPA2 needs 8..63.
      size_t ls = strlen(argv[1]), lp = nargs == 2 ? strlen(argv[2]) : 0;
      if (nargs > 2 || ls < 1 || ls > 32 || (nargs == 2 && (lp < 8 || lp > 63))) {
        replyBadArgs();
        return;
      }
      strcpy(net_ssid, argv[1]);
      strcpy(net_pass, nargs == 2 ? argv[2] : "");
      saveNet();
      out->println("OK");  // before the reconnect, in case this came over TCP 8900
      netStart();
#else
      out->println("ERR no wifi");
#endif
      return;
    }

    case 'A': {
#ifdef AUDIO_ENABLED
      if (nargs == 0) {
        out->printf("AUDIO rate=%d shift=%u clients=%d frames=%lu dropped=%lu i2s_timeouts=%lu\n",
                    AUD_RATE, (unsigned)aud_shift, aud_client_on ? 1 : 0,
                    (unsigned long)aud_frames, (unsigned long)aud_dropped,
                    (unsigned long)aud_timeouts);
        return;
      }
      // A shift <8..16>: whole numbers only. Lower = louder, clips sooner.
      if (nargs != 2 || strcasecmp(argv[1], "shift") != 0 || !parseF(argv[2], &a) ||
          a != floorf(a) || a < AUD_SHIFT_MIN || a > AUD_SHIFT_MAX) {
        replyBadArgs();
        return;
      }
      aud_shift = (uint8_t)a;  // the capture task picks it up on its next frame
      prefs.begin(NVS_NAMESPACE, false);
      prefs.putUChar("aud_shift", aud_shift);
      prefs.end();
      out->println("OK");
#else
      out->println("ERR no audio");
#endif
      return;
    }

    default:
      out->println("ERR unknown cmd");
      return;
  }
}

// Feed one received byte of a host link into its line assembler; a complete
// line is handled with its replies going back to that same link.
static void feedLine(LineBuf &lb, int ch, Print *link) {
  if (ch == '\n' || ch == '\r') {
    out = link;
    if (lb.overflow) {
      out->println("ERR line too long");
    } else {
      lb.buf[lb.len] = '\0';
      handleLine(lb.buf);
    }
    out         = &Serial;
    lb.len      = 0;
    lb.overflow = false;
  } else if (lb.len < CMD_LINE_MAX - 1) {
    lb.buf[lb.len++] = (char)ch;
  } else {
    lb.overflow = true;
  }
}

// ----------------------------------------------------------------- arduino --

void setup() {
  Serial.setRxBufferSize(512);
  Serial.begin(SERIAL_BAUD);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  // Serial is the native USB (HWCDC) port here. Cap how long a reply may wait
  // for the Mac, so a host that stopped reading cannot stall the loop and,
  // with it, the LiDAR pass-through.
  Serial.setTxTimeoutMs(10);
#endif

  loadCal();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(SERVO_HZ);
  // Attach with the library's absolute bounds; degToUs() clamps to the
  // calibrated window, so `C` never needs a detach/re-attach.
  servo.attach(SERVO_PIN, US_ABS_MIN, US_ABS_MAX);
  // Must come after attach(): attach() on a fresh pin resets the width to the
  // 10-bit default. setTimerWidth() re-attaches with the new resolution.
  servo.setTimerWidth(SERVO_TIMER_BITS);

  // Known pose at boot so `?` is truthful from the first command.
  applyAngle(clampTarget((float)cal.range / 2.0f));

  encInit();
#ifdef LIDAR_PASSTHROUGH
  lidarInit();
#ifdef NET_ENABLED
  tcpRingInit();
#endif
#endif
#ifdef NET_ENABLED
  loadNet();
  netStart();  // always starts the SLAM-Test access point; joins a stored network too
#endif
#ifdef AUDIO_ENABLED
#if !AUDIO_CLOCK_JUMPERS
  // pin_connection.md: GPIO8/9 stay unconnected, plain inputs.
  pinMode(AUD_SPARE0_PIN, INPUT);
  pinMode(AUD_SPARE1_PIN, INPUT);
#endif
  aud_on = audInit();
  if (aud_on) {
    aud_server.setNoDelay(true);
    aud_server.begin();
  }
#endif

  delay(50);
  // Note lines before the banner (the core discards boot output anyway).
#ifdef LIDAR_PASSTHROUGH
  Serial.printf("NOTE lidar pass-through: %s <-> UART1 (RX GPIO%d, TX GPIO%d) at %d\n",
                LIDAR_HOST_DESC, LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_BAUD);
#else
  Serial.println("NOTE lidar pass-through not built: needs the ESP32-S3");
#endif
#ifdef NET_ENABLED
  Serial.printf("NOTE wifi: access point %s at 192.168.4.1\n", AP_SSID);
  if (net_ssid[0]) Serial.printf("NOTE wifi: joining %s as %s.local\n", net_ssid, NET_HOSTNAME);
  else             Serial.println("NOTE wifi: no network stored, set one with N <ssid> <password>");
#endif
#ifdef AUDIO_ENABLED
  if (aud_on) Serial.printf("NOTE audio: 4 mics at %d Hz on TCP %d, I2S1 clock via %s, ring %lu frames\n",
                            AUD_RATE, AUD_PORT, aud_clock_desc, (unsigned long)aud_ring_frames);
  else        Serial.println("NOTE audio: init failed, audio=off");
#else
  Serial.println("NOTE audio not built: needs the ESP32-S3");
#endif
  Serial.println(BANNER);
}

void loop() {
  service();
#ifdef NET_ENABLED
  netLoop();
#endif

  while (Serial.available() > 0) {
    int ch = Serial.read();
    if (ch < 0) break;
    feedLine(serial_line, ch, &Serial);
  }

#ifdef NET_ENABLED
  if (tilt_client_on) {
    uint8_t buf[64];
    int n = tilt_client.available();
    if (n > 0) {
      n = tilt_client.read(buf, n < (int)sizeof(buf) ? (size_t)n : sizeof(buf));
      for (int i = 0; i < n; i++) feedLine(tilt_line, buf[i], &tilt_client);
    }
  }
#endif
}

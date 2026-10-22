// servo_tilt.ino -- OBSLAM payload: tilt-servo controller
//
// Drives one GS2010MG 180-degree servo (50 Hz PWM) that tilts the LiDAR cradle,
// reads the cradle angle back from an AS5600 magnetic encoder on the tilt axle
// (I2C; optional, a board without one behaves exactly like the pre-encoder
// firmware), and talks to the core server over USB serial. The protocol is the
// contract in docs/api.md, section 2:
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
// Boards: plain ESP32 DevKit (esp32:esp32:esp32) and ESP32-S3 DevKitC-1
// (esp32:esp32:esp32s3). Compiles for both. See firmware/README.md.

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <Wire.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

// ---------------------------------------------------------------- settings --

#define SERVO_PIN        13        // GS2010MG signal (yellow/white)
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
#define CMD_LINE_MAX     64        // longest accepted command line, incl. NUL

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
#define AS5600_SDA       8
#define AS5600_SCL       9
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

static char   line[CMD_LINE_MAX];
static size_t line_len      = 0;
static bool   line_overflow = false;

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

// ------------------------------------------------------------------- servo --

// deg must already be clamped.
static void applyAngle(float deg) {
  int us = degToUs(deg);
  if (us != cur_us) settleReset();  // a new pulse: the cradle is about to move
  servo.writeMicroseconds(us);
  cur_us  = us;
  cur_deg = deg;
}

// Wait ms while keeping the encoder polled.
static void waitPolling(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    encPoll();
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
    Serial.printf("OK %s %s\n", a, m);
  } else {
    Serial.printf("OK %s\n", a);
  }
}

static void replyStatus() {
  char a[16], m[16], lo[16], hi[16];
  fmtDeg(a,  sizeof(a),  cur_deg);
  fmtMeas(m, sizeof(m));
  fmtDeg(lo, sizeof(lo), cal.lim_min);
  fmtDeg(hi, sizeof(hi), cal.lim_max);
  Serial.printf("STATUS angle=%s meas=%s min=%s max=%s range=%d us_min=%d us_max=%d enc=%s settled=%d\n",
                a, m, lo, hi, cal.range, cal.us_min, cal.us_max,
                ENC_STATUS_STR[enc.status], (enc.present && settled) ? 1 : 0);
}

// Magnet-spacing diagnostics. raw and deg are the chip's unzeroed reading;
// zero is on that same scale, so meas = dir * (deg - zero), wrapped.
static void replyEnc() {
  char d[16], z[16];
  if (enc.present && enc.valid) fmtDeg(d, sizeof(d), encRawDeg());
  else                          snprintf(d, sizeof(d), "nan");
  fmtDeg(z, sizeof(z), enc.zero);
  Serial.printf("ENC raw=%u deg=%s zero=%s dir=%d status=%s\n",
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
  cal.lim_min = prefs.getFloat("lim_min", 0.0f);
  cal.lim_max = prefs.getFloat("lim_max", (float)cal.range);
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

static void replyBadArgs() { Serial.println("ERR bad args"); }

// E zero / E dir need a reading with a magnet behind it.
static bool encUsable() {
  if (!enc.present) {
    Serial.println("ERR no encoder");
    return false;
  }
  if (!enc.valid || enc.status == ENC_ERR || enc.status == ENC_NOMAG) {
    Serial.printf("ERR enc=%s\n", ENC_STATUS_STR[enc.status]);
    return false;
  }
  return true;
}

static void handleLine(char *s) {
  char *argv[6];
  int   argc = tokenize(s, argv, 6);
  if (argc == 0) return;  // blank line: no reply
  if (argc > 4 || strlen(argv[0]) != 1) {
    Serial.println(argc > 4 ? "ERR bad args" : "ERR unknown cmd");
    return;
  }

  char cmd   = (char)toupper((unsigned char)argv[0][0]);
  int  nargs = argc - 1;
  float a = 0, b = 0, c = 0;

  switch (cmd) {
    case 'I':
      if (nargs != 0) { replyBadArgs(); return; }
      Serial.println(BANNER);
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
        delay(WAIT_NOENC_MS);
        fmtDeg(m, sizeof(m), cur_deg);
        Serial.printf("OK %s settled=1\n", m);
        return;
      }
      unsigned long budget = (unsigned long)ms;
      unsigned long t0     = millis();
      while (!settled && millis() - t0 < budget) {
        encPoll();
        delay(1);
      }
      fmtMeas(m, sizeof(m));
      Serial.printf("OK %s settled=%d\n", m, settled ? 1 : 0);
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
      Serial.print("OK ");
      Serial.println(m);
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
      Serial.println("OK");
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
      Serial.println("OK");
      return;
    }

    default:
      Serial.println("ERR unknown cmd");
      return;
  }
}

// ----------------------------------------------------------------- arduino --

void setup() {
  Serial.setRxBufferSize(512);
  Serial.begin(SERIAL_BAUD);

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

  delay(50);
  Serial.println(BANNER);
}

void loop() {
  encPoll();

  while (Serial.available() > 0) {
    int ch = Serial.read();
    if (ch < 0) break;

    if (ch == '\n' || ch == '\r') {
      if (line_overflow) {
        Serial.println("ERR line too long");
      } else {
        line[line_len] = '\0';
        handleLine(line);
      }
      line_len      = 0;
      line_overflow = false;
    } else if (line_len < CMD_LINE_MAX - 1) {
      line[line_len++] = (char)ch;
    } else {
      line_overflow = true;
    }
  }
}

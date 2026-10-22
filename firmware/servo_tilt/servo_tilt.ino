// servo_tilt.ino -- OBSLAM payload: tilt-servo controller
//
// Drives one DS3218 servo (50 Hz PWM) that tilts the LiDAR cradle and talks to
// the core server over USB serial. The protocol is the contract in
// docs/api.md, section 2:
//
//   115200 8N1, '\n'-terminated ASCII lines ('\r' is accepted as a terminator
//   too, so `screen` works), exactly one reply line per command. Angles are
//   servo degrees 0..RANGE (RANGE = 180 or 270).
//
//   I                           -> OBSLAM-TILT v1
//   ?                           -> STATUS angle=<deg> min=<deg> max=<deg> range=<180|270> us_min=<us> us_max=<us>
//   T <deg>                     -> OK <deg>   move now, clamped, reply once the pulse is set
//   S <deg> <ms>                -> OK <deg>   ease linearly over <ms>, reply when done
//   H                           -> OK <deg>   go to RANGE/2
//   L <min> <max>               -> OK         soft limits (deg), persisted in NVS
//   C <us_min> <us_max> <range> -> OK         pulse calibration, persisted in NVS
//   anything else               -> ERR <msg>
//
// Boards: plain ESP32 DevKit (esp32:esp32:esp32) and ESP32-S3 DevKitC
// (esp32:esp32:esp32s3). Compiles for both. See firmware/README.md.

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------- settings --

#define SERVO_PIN        13        // DS3218 signal (yellow/white)
#define SERVO_HZ         50

// ESP32Servo defaults to a 10-bit timer (19.5 us per tick, ~2.6 deg on a 270
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
#define CMD_LINE_MAX         64        // longest accepted command line, incl. NUL

// Pulse bounds the ESP32Servo library itself enforces (MIN/MAX_PULSE_WIDTH).
// `C` refuses anything outside these.
#define US_ABS_MIN       500
#define US_ABS_MAX       2500

// Factory defaults from the contract: 500 2500 270, soft limits = full range.
#define DEF_US_MIN       500
#define DEF_US_MAX       2500
#define DEF_RANGE        270

#define NVS_NAMESPACE    "tilt"

// ------------------------------------------------------------------- state --

struct Cal {
  int   us_min;   // pulse at 0 deg
  int   us_max;   // pulse at RANGE deg
  int   range;    // 180 or 270
  float lim_min;  // soft limit, servo degrees
  float lim_max;
};

static Servo       servo;
static Preferences prefs;
static Cal         cal;
static float       cur_deg = 0.0f;  // last commanded angle; the servo has no feedback

static char   line[CMD_LINE_MAX];
static size_t line_len      = 0;
static bool   line_overflow = false;

// ----------------------------------------------------------------- helpers --

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
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

// deg must already be clamped.
static void applyAngle(float deg) {
  servo.writeMicroseconds(degToUs(deg));
  cur_deg = deg;
}

// Linear ease from the current angle to target over ms (already validated).
static void easeTo(float target, float ms) {
  float start = cur_deg;
  int   n     = (int)(ms / EASE_STEP_MS);
  if (n < 1) n = 1;
  unsigned long step_ms = (unsigned long)(ms / (float)n);
  for (int i = 1; i <= n; i++) {
    applyAngle(start + (target - start) * ((float)i / (float)n));
    delay(step_ms);
  }
  applyAngle(target);  // land exactly, regardless of float rounding
}

// "135", "10.25", "0" -- up to two decimals, trailing zeros trimmed.
static void fmtDeg(char *out, size_t n, float deg) {
  snprintf(out, n, "%.2f", deg);
  char *dot = strchr(out, '.');
  if (!dot) return;
  char *end = out + strlen(out) - 1;
  while (end > dot && *end == '0') *end-- = '\0';
  if (*end == '.') *end = '\0';
}

static void replyOkDeg(float deg) {
  char buf[16];
  fmtDeg(buf, sizeof(buf), deg);
  Serial.print("OK ");
  Serial.println(buf);
}

static void replyStatus() {
  char a[16], lo[16], hi[16];
  fmtDeg(a,  sizeof(a),  cur_deg);
  fmtDeg(lo, sizeof(lo), cal.lim_min);
  fmtDeg(hi, sizeof(hi), cal.lim_max);
  Serial.printf("STATUS angle=%s min=%s max=%s range=%d us_min=%d us_max=%d\n",
                a, lo, hi, cal.range, cal.us_min, cal.us_max);
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

static void loadCal() {
  prefs.begin(NVS_NAMESPACE, false);
  cal.us_min  = prefs.getInt("us_min", DEF_US_MIN);
  cal.us_max  = prefs.getInt("us_max", DEF_US_MAX);
  cal.range   = prefs.getInt("range", DEF_RANGE);
  cal.lim_min = prefs.getFloat("lim_min", 0.0f);
  cal.lim_max = prefs.getFloat("lim_max", (float)cal.range);
  prefs.end();
  sanitizeCal();
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

  delay(50);
  Serial.println(BANNER);
}

void loop() {
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

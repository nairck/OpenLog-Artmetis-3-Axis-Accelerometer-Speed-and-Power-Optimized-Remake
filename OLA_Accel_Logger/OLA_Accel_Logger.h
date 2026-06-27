/*
  OLA_Accel_Logger.h  v1.0  — implementation: configuration, runtime state,
  hardware helpers, the event-capture engine, the serial menu, and the 3-wire
  partner cross-trigger. The companion .ino holds only setup() and loop().
  See the .ino header for the feature/output/menu overview.
*/

#pragma once

// -- Includes -----------------------------------------------------------------
#include <EEPROM.h>
#include <SPI.h>
#include <SdFat.h>
#include "RTC.h"
#include "ICM_20948.h"
#include "am_mcu_apollo.h"   // for am_hal_burst_mode_* (96 MHz burst mode)

// -- Firmware identity --------------------------------------------------------
#define FW_MAJOR 1
#define FW_MINOR 0
#define OLA_IDENTIFIER 0x1B7

// Compile timestamp — changes every build. Stored in EEPROM after Settings;
// if it doesn't match, this is a fresh flash and compiled defaults are applied.
static const char BUILD_STAMP[] = __DATE__ " " __TIME__;
#define BUILD_STAMP_ADDR ((int)sizeof(Settings) + 8)
#define BUILD_STAMP_LEN  ((int)sizeof(BUILD_STAMP))

// -- Hardware pins (v1.0 red board) -------------------------------------------
const byte PIN_MICROSD_CHIP_SELECT = 23;
const byte PIN_IMU_POWER           = 27;
const byte PIN_PWR_LED             = 29;   // "power" LED (used here as the red status flash)
const byte PIN_POWER_LOSS          = 3;
const byte PIN_MICROSD_POWER       = 15;
const byte PIN_QWIIC_POWER         = 18;
const byte PIN_STAT_LED            = 19;   // "stat" LED (used here as the blue handshake flash)
const byte PIN_IMU_INT             = 37;
const byte PIN_IMU_CHIP_SELECT     = 44;
const byte PIN_SPI_CIPO            = 6;

// Cross-trigger 3-wire partner link (out, in, shared ground). ACTIVE HIGH:
// TRIG_OUT idles LOW and is driven HIGH only while THIS unit's own event window
// is live; TRIG_IN reads HIGH while the partner's window is live. These are two
// of the four edge pads the OLA breaks out and are otherwise unused.
const byte PIN_TRIG_OUT = 32;   // our event-active output (push-pull, idle LOW)
const byte PIN_TRIG_IN  = 11;   // partner event-active input (interrupt + pulldown)

// -- ICM-20948 register addresses ---------------------------------------------
#define ICM_REG_BANK_SEL   0x7F
#define ICM_REG_ACCEL_XOUT 0x2D   // ACCEL_XOUT_H, Bank 0
#define ICM_BANK0          0x00
#define ICM_SPI_HZ         7000000

// -- SD -----------------------------------------------------------------------
#define SD_FAT_TYPE 3
#define SD_CONFIG SdSpiConfig(PIN_MICROSD_CHIP_SELECT, SHARED_SPI, SD_SCK_MHZ(24))
SdFs   sd;
FsFile dataFile;
char   dataFileName[30] = "";

// -- RTC / IMU ----------------------------------------------------------------
Apollo3RTC    myRTC;
ICM_20948_SPI myICM;

// -- Cached SPI settings for the IMU (constructed once, reused). --------------
static const SPISettings ICM_SPI_SETTINGS(ICM_SPI_HZ, MSBFIRST, SPI_MODE0);

// -- Settings -----------------------------------------------------------------
struct Settings {
  int      sizeOfSettings        = 0;
  int      olaIdentifier         = OLA_IDENTIFIER;
  int      nextLogNumber         = 1;
  uint32_t usBetweenReadings     = 0;             // 0 = max rate
  bool     enableTerminalOutput  = false;
  bool     imuAccDLPF            = false;
  int      imuAccFSS             = 3;             // 0=2g 1=4g 2=8g 3=16g
  int      imuAccDLPFBW          = 7;
  bool     recordEventsOnly      = true;          // event-triggered capture
  uint32_t eventWindowMs         = 3000;          // total window saved (20/80 split)
  float    eventThreshMg         = 1000.0f;       // mg; deviation from 1g that triggers (0.1-16000)
  uint8_t  evPrePct              = 20;            // pre-event % of window (2-98, default 20)
  uint32_t openNewLogFilesAfter  = 86400;         // s; min=60; max=604800 (1 wk)
  uint32_t maxLogFileBytes       = 1000000000UL;  // bytes; min 25KB; max 3.8 GB
  bool     burstEnabled          = true;          // 96 MHz TurboSPOT (option 9)
  int      terminalBaudRate      = 115200;
  uint8_t  ledMode               = 2;             // 0=normal 1=flash15s 2=after-event 3=off 
  uint8_t  partnerMode           = 1;             // 0=OFF 1=AUTO (3-wire cross-trigger)
} settings;

#define MAX_LOGFILE_BYTES_CAP   3800000000UL     // hard ceiling for size rotation = 3.8 GB
#define MAX_LOGFILE_SECONDS_CAP 604800UL         // hard ceiling for duration rotation = 1 week

// -- Event-capture ring buffer ------------------------------------------------
// The ring length dictates the captured-window size: a window extends until the
// event ends OR the ring fills, at which point the whole buffer is flushed (its
// own DAY marker) and a fresh window continues. No fixed millisecond ceiling.
#define EVENT_RING_SAMPLES 38000
static uint16_t evDelta[EVENT_RING_SAMPLES];   // per-sample us delta
static int16_t  evAx[EVENT_RING_SAMPLES];
static int16_t  evAy[EVENT_RING_SAMPLES];
static int16_t  evAz[EVENT_RING_SAMPLES];
// Anchors (absolute micros): evStartMicros = the sample at evCapStart (first in
// the chunk) - used for the DAY marker and the trim age; evLastMicros = the most
// recently stored sample - used to form each uint16 delta (m - evLastMicros).
static uint32_t evStartMicros = 0;
static uint32_t evLastMicros  = 0;

// -- Misc timing constants ----------------------------------------------------
// -- Rate constants (used in menus, ring estimate, startup check) -----------
#define RATE_BOOST_HZ  5400        // approximate peak Hz with 96 MHz TurboSPOT
#define RATE_NORMAL_HZ 3200        // approximate peak Hz at 48 MHz
#define MIN_EVENT_RATE_HZ  20      // 20 Hz floor for event mode (16-bit delta safety)

// -- Limit constants --------------------------------------------------------
#define MIN_WINDOW_MS    200       // minimum event window (ms)
#define MIN_THRESH_MG    100.0f    // minimum threshold (mg)
#define MAX_THRESH_MG    16000.0f  // maximum threshold (mg) = full 16g range
#define MIN_FILE_SECONDS 60        // 1 minute minimum for time-based rotation
#define MIN_FILE_BYTES   25000UL   // 25 KB minimum for size-based rotation

#define HPF_SETTLE_MS  2000        // ignore the OWN threshold this long after logging

// --- Partner toggle-handshake (pairing) -------------------------------------
// The 3-wire link carries TWO signals: a STEADY HIGH (>1 ms) = the partner is in
// an event (cross-trigger), and a fast TOGGLE (250 us on/off) = the partner is
// pairing. A HIGH that falls within HS_PULSE_MAX_US is a pairing pulse, not an
// event. Detection is interrupt-driven (never polled) so an edge is never missed.
#define HS_TOGGLE_US        250UL      // toggle half-period (250 us HIGH, 250 us LOW)
#define HS_PULSE_MAX_US     1000UL     // HIGH shorter than this, then LOW = a pairing pulse
#define HS_STOP_QUIET_US    1000UL     // partner line quiet this long = it stopped toggling
#define HS_MIN_RESP_US      500UL      // responder toggles >= this (>=2 pulses) before "quiet" counts
#define HS_RESP_TIMEOUT_US  2000000UL  // responder/handshake gives up after 2 s
#define HS_PULSES_CONFIRM   2          // # pairing pulses confirming the partner is really toggling
#define PARTNER_HOLD_MS     5000UL     // suppress event starts this long after a handshake
#define DAY_RESET_THRESH_US 8640000ULL // 0.0001 day; >= this on connect -> split the log file
#define HS_BLINK_TOGGLE_MS  60UL       // pairing-confirm flash: toggle period (ms) - lower = faster
#define HS_BLINK_TOTAL_MS   500UL      // pairing-confirm flash: total duration (ms)
#define HOLD_END_ON_MS      20UL       // hold-end "logging live" flash: LED on time (ms); short = dim/low-power
#define HOLD_END_OFF_MS     80UL       // hold-end flash: LED off time (ms)
#define HOLD_END_FLASH_MS   1000UL     // hold-end flash: total duration (ms)
                                   //   begins so gravity removal can't fire a false event
#define AFTER_FLASH_MS   10        // brief STAT blip after an event window closes (ledMode 2)
#define RATE_RING_N      20        // samples used for the live rate estimate in the menu

// -- Runtime state ------------------------------------------------------------
static bool     sdOnline    = false;
static bool     imuOnline   = false;
static bool     useRawRead  = false;
static bool     useGpioReady = false;
static bool     burstMode   = false;
static bool     burstInitDone  = false;
static bool     burstAvailable = false;
static uint32_t lastWriteMicros    = 0;
static uint32_t lastRotateMillis   = 0;
static uint32_t lastLedFlashMillis = 0;
static uint32_t sampleCount        = 0;
static uint32_t bootMillis         = 0;
static uint64_t bootMicros         = 0;       // 64-bit so the DAY clock can be re-zeroed
static uint32_t fileBytesWritten   = 0;
static uint32_t rotateAfterMillis  = 86400000UL;

// micros() wrap tracking for the decimal-day counter.
static uint32_t usHigh    = 0;
static uint32_t prevMicros = 0;

// Latest raw accel (set by readAccel; used by event mode + threshold).
static int16_t  rawAx = 0, rawAy = 0, rawAz = 0;

// Event-capture state machine.
//   selfActive    : our OWN threshold-defined window is live -> drives TRIG_OUT.
//   recordingPrev : was the unit recording (self OR partner) on the last sample.
//   selfPostUntil : micros() deadline of our own post-window.
// Recording (buffering to the ring) = selfActive OR partner; TRIG_OUT mirrors
// selfActive ONLY, so two units can never latch each other on.
static uint32_t evHead            = 0;
static uint32_t evCapStart        = 0;
static uint32_t evCapLen          = 0;
static bool     selfActive        = false;
static bool     recordingPrev     = false;
static uint32_t selfPostUntil     = 0;
static uint32_t evPreWindowUs     = 800000UL;    // pre-event span (evPrePct of window), us, cached
static uint32_t evPostWindowUs    = 3200000UL;   // post-event span (rest of window), us, cached
static uint32_t evMagHigh2        = 4194304UL;   // thresh in raw^2, cached

// Cross-trigger partner link runtime state.
static bool     partnerSeen       = false;       // sticky: a partner HIGH has been seen (set in ISR)
volatile bool   partnerLineHigh   = false;       // partner TRIG_IN level, latched by ISR
volatile uint32_t partnerRiseMicros = 0;         // micros of the last rising edge on TRIG_IN
volatile uint32_t partnerEdgeMicros = 0;         // micros of the last ANY edge (toggling-stop detect)
volatile uint16_t partnerPulseCount = 0;         // # of HIGH->LOW-within-1ms pairing pulses seen
volatile bool   partnerHsPulse    = false;       // sticky: a pairing pulse was seen (hot-loop trigger)
static bool     partnerPaired     = false;       // partner handshake completed (partner mode live)
static uint32_t partnerHoldUntil  = 0;           // millis() until event starts are allowed again

// HPF threshold settling: after logging begins, suppress the OWN threshold for
// HPF_SETTLE_MS while the high-pass keeps updating (gravity removal settles).
// Partner triggering is NOT suppressed during settling.
static bool     hpfSettling       = true;
static uint32_t logStartMillis    = 0;

// Live sample-rate estimate shown in the menu: ring of the last RATE_RING_N
// sample timestamps (filled every loop). Cheap: one store + compare-increment.
static uint32_t rateRing[RATE_RING_N];
static uint8_t  rateRingIdx       = 0;
static uint8_t  rateRingCount     = 0;

// Non-blocking after-event flash (ledMode 3): a short STAT blip when a window
// closes, turned back off by serviceLeds() after AFTER_FLASH_MS.
static bool     afterFlashOn      = false;
static uint32_t afterFlashStart   = 0;

// One-shot blue "partner detected" blink (non-blocking, ~1 s at 120 ms toggle).
// hsFlashArmed waits for the first LIVE partner event when there was no startup
// handshake; hsFlashRunning is the blink in progress, driven by serviceLeds().
static bool     hsFlashArmed      = false;
static bool     hsFlashRunning    = false;
static uint32_t hsFlashStart      = 0;
static uint32_t hsFlashToggle     = 0;
static bool     hsFlashState      = false;

// Hold-end "logging live" flash: when the post-handshake hold expires, a brief
// low-duty STAT pulse train marks logging going live, synchronized across both
// units (their holds end together). Driven (non-blocking) by serviceLeds().
static bool     holdEndFlashRunning = false;
static uint32_t holdEndFlashStart   = 0;
static uint32_t holdEndFlashToggle  = 0;
static bool     holdEndFlashState   = false;

// True if the current / just-closed event window involved the partner line.
static bool     evPartnerInWindow = false;

// Single-pole IIR high-pass for threshold only.
#define HP_ALPHA_Q16 3
static int32_t hpLpAx = 0, hpLpAy = 0, hpLpAz = 0;

#define SPIN_GUARD_MAX 2000000UL

// -- SD write batching --------------------------------------------------------
#define SDBUF_BYTES        32768
#define SD_WRITE_THRESHOLD 16384
#define SYNC_EVERY_SECTORS 512
static uint8_t  sdBuf[SDBUF_BYTES];
static uint16_t sdBufLen = 0;
static uint32_t sectorsSinceSync = 0;

// -- Accel scale --------------------------------------------------------------
static int32_t accelMulQ16 = 3200000;
static int32_t accelX100X = 0, accelX100Y = 0, accelX100Z = 0;

const uint16_t MENU_TIMEOUT_SEC = 420;

// -- ISRs ---------------------------------------------------------------------
volatile bool powerLossSeen = false;
void powerLossISR() { powerLossSeen = true; }

// Partner trigger-in edge (active HIGH). The CHANGE interrupt latches the line
// level into a RAM flag (partnerLineHigh) so the hot loop never polls GPIO, and
// it fires even while the loop is blocked in an SD flush, so a partner edge is
// never missed. Reading the real level on every edge is self-correcting; the
// clean push-pull signal needs no debounce.
void partnerISR() {
  uint32_t now = micros();
  bool hi = (digitalRead(PIN_TRIG_IN) == HIGH);
  partnerEdgeMicros = now;                        // any edge: feeds the toggling-stop test
  if (hi) {
    partnerRiseMicros = now;
    partnerSeen = true;                           // sticky: a partner has driven the line HIGH
  } else if ((uint32_t)(now - partnerRiseMicros) < HS_PULSE_MAX_US) {
    partnerPulseCount++;                          // a brief HIGH then LOW = a pairing pulse...
    partnerHsPulse = true;                        // ...not an event; flagged for the hot loop
  }
  partnerLineHigh = hi;                           // level latch for the normal cross-trigger
}

// -- Hardware helpers ---------------------------------------------------------
static inline void imuPowerOn()    { digitalWrite(PIN_IMU_POWER,     HIGH); }
static inline void imuPowerOff()   { digitalWrite(PIN_IMU_POWER,     LOW);  }
static inline void sdPowerOn()     { digitalWrite(PIN_MICROSD_POWER, LOW);  }
static inline void qwiicPowerOff() { digitalWrite(PIN_QWIIC_POWER,   LOW);  }
static inline void statLedOn()     { digitalWrite(PIN_STAT_LED,      HIGH); }
static inline void statLedOff()    { digitalWrite(PIN_STAT_LED,      LOW);  }
static inline void pwrLedOn()      { digitalWrite(PIN_PWR_LED,       HIGH); }
static inline void pwrLedOff()     { digitalWrite(PIN_PWR_LED,       LOW);  }

// Cross-trigger output (active HIGH = our event live). Driven only on self-event
// transitions, and only while partner mode is AUTO.
static inline void trigOutHigh()   { digitalWrite(PIN_TRIG_OUT,      HIGH); }
static inline void trigOutLow()    { digitalWrite(PIN_TRIG_OUT,      LOW);  }

// Partner mode AUTO? (0=OFF, 1=AUTO). AUTO = listen for a partner + drive our
// trig-out during our own events, auto-connecting a partner at any time.
static inline bool partnerAuto()   { return settings.partnerMode == 1; }

// In the PARTNER_HOLD_MS window right after a handshake? Used to suppress event
// starts AND to ignore the partner's residual pairing toggles, so its echo can't
// re-fire the connect handler (that would re-arm the hold and double the confirm /
// hold-end flashes). millis() is read only when a pulse is pending, never in the
// steady-state hot path.
static inline bool inPartnerHold() {
  return (partnerHoldUntil != 0) && ((int32_t)(millis() - partnerHoldUntil) < 0);
}

// Arm the post-handshake hold so it ends exactly PARTNER_HOLD_MS after the handshake
// COMPLETION instant (completeRaw - captured the moment the toggling stops: this unit
// the instant it drops LOW, the partner ~1 ms later when it confirms quiet), NOT after
// the startup/LED/file work that follows it. Both units anchor to that same instant
// (to ~1 ms), so the 'logging live' flash lands within ~1 ms on the pair no matter how
// much boot work ran afterward. micros() - completeRaw is < 1 wrap (a few seconds).
static inline void armPartnerHold(uint64_t completeRaw) {
  uint32_t sinceUs = (uint32_t)micros() - (uint32_t)completeRaw;   // elapsed since the handshake
  uint32_t nowMs   = millis();
  if (sinceUs >= PARTNER_HOLD_MS * 1000UL) {                       // handshake already older than the hold
    partnerHoldUntil = nowMs ? nowMs : 1;                          // -> let the flash fire immediately
  } else {
    partnerHoldUntil = nowMs + (PARTNER_HOLD_MS - sinceUs / 1000UL);
    if (partnerHoldUntil == 0) partnerHoldUntil = 1;               // 0 is the 'no hold' sentinel; avoid it
  }
}

// Effective sample rate (Hz) for the ring-length estimate. Peak is RATE_NORMAL_HZ
// in normal mode, RATE_BOOST_HZ in burst; a slower SET rate caps it below that.
static inline uint32_t effectiveSampleRateHz() {
  uint32_t peak = settings.burstEnabled ? RATE_BOOST_HZ : RATE_NORMAL_HZ;
  if (settings.usBetweenReadings > 0) {
    uint32_t setHz = (uint32_t)(1000000UL / settings.usBetweenReadings);
    if (setHz < peak) return setHz;
  }
  return peak;
}
// Estimated ring-buffer span in seconds at the effective rate (caps the window).
static inline float estimateRingSeconds() {
  uint32_t hz = effectiveSampleRateHz();
  if (hz == 0) hz = 1;
  return (float)EVENT_RING_SAMPLES / (float)hz;
}

// Non-blocking LED servicing, called at the top of every loop iteration.
//  - turns the short after-event flash (ledMode 3) back off after AFTER_FLASH_MS
//  - drives the one-shot ~1 s blue "partner detected" blink (120 ms toggle)
// The blue blink takes priority over the after-event blip on the shared STAT LED.
static inline void serviceLeds() {
  uint32_t now = millis();
  if (holdEndFlashRunning) {                                 // hold-end flash owns STAT while running
    if ((uint32_t)(now - holdEndFlashStart) >= HOLD_END_FLASH_MS) { statLedOff(); holdEndFlashRunning = false; }
    else if ((int32_t)(now - holdEndFlashToggle) >= 0) {
      holdEndFlashState = !holdEndFlashState;
      if (holdEndFlashState) { statLedOn();  holdEndFlashToggle = now + HOLD_END_ON_MS;  }
      else                   { statLedOff(); holdEndFlashToggle = now + HOLD_END_OFF_MS; }
    }
    return;                                                  // priority over after/hs flash on the shared STAT
  }
  if (afterFlashOn && (uint32_t)(now - afterFlashStart) >= AFTER_FLASH_MS) {
    if (!hsFlashRunning) statLedOff();
    afterFlashOn = false;
  }
  if (hsFlashRunning) {
    if ((uint32_t)(now - hsFlashStart) >= 1000UL) { statLedOff(); hsFlashRunning = false; }
    else if ((int32_t)(now - hsFlashToggle) >= 0) {
      hsFlashState = !hsFlashState;
      hsFlashState ? statLedOn() : statLedOff();
      hsFlashToggle = now + 120UL;
    }
  }
}

// Forward declares needed by startupRateCheck (defined later in the file).
static inline void readAccel(bool scale);
static inline bool imuDataReady();

// Startup rate check: sample for 1 s and verify the rate reaches >= 90% of expected.
// Called once from setup() right after logging begins, during HPF settling.
// If the rate is below the threshold, flash the LED and force a system restart.
static void startupRateCheck() {
  uint32_t expectedHz = effectiveSampleRateHz();
  if (expectedHz < MIN_EVENT_RATE_HZ) expectedHz = MIN_EVENT_RATE_HZ;
  // For the check, sample at the user's rate but at least 20 Hz
  uint32_t checkUs = settings.usBetweenReadings;
  uint32_t minUs   = 1000000UL / MIN_EVENT_RATE_HZ;
  if (checkUs > minUs) checkUs = minUs;            // slow rate -> check at 20 Hz
  // checkUs == 0 is fine: max rate
  Serial.print(F("Startup rate check (1 s, expecting >= "));
  Serial.print((expectedHz * 9u) / 10u);
  Serial.print(F(" Hz) .........."));
  uint32_t startMs = millis();
  uint32_t count = 0;
  uint32_t lastCheck = micros();
  while ((uint32_t)(millis() - startMs) < 1000) {
    if (checkUs > 0 && (uint32_t)(micros() - lastCheck) < checkUs) continue;
    uint32_t sg = 0;
    while (!imuDataReady()) { if (++sg >= SPIN_GUARD_MAX) break; }
    if (sg >= SPIN_GUARD_MAX) continue;
    readAccel(false);
    lastCheck = micros();
    count++;
  }
  uint32_t minHz = (expectedHz * 9u) / 10u;       // 90% of expected
  bool failCheck = false;
  if (count < minHz) {
    Serial.println(F(" *** FAIL: rate too low - restarting in 1 s ***"));     failCheck = true;
  } else {
    Serial.println(F(" PASS"));
  }
  Serial.print(F("  Measured: ")); Serial.print(count);
  Serial.print(F(" samples = ")); Serial.print(count);
  Serial.print(F(" Hz (expected: ")); Serial.print(expectedHz);
  Serial.print(F(" Hz, pass >= ")); Serial.print(minHz); Serial.println(F(")"));
  if (failCheck == true) {
    pwrLedOn(); delay(1000); NVIC_SystemReset();
  }
}

// Forward declares (used before their definitions below).
static void emergencyFlush();
void        recordSettings();
static bool startupLedSequence(uint64_t *completeRaw, bool announce);
static void blinkLed(byte pin, uint16_t toggleMs, uint16_t totalMs);
static void rotateLogFile();
static void applyBurstMode();
static void applyPartnerMode();
static void attachPartnerInterrupt();

static inline uint8_t u32toa(uint32_t v, char *buf) {
  if (v == 0) { buf[0] = '0'; return 1; }
  char tmp[10]; uint8_t n = 0;
  while (v) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
  uint8_t i = 0;
  do { buf[i] = tmp[n-1-i]; i++; } while (i < n);
  return n;
}
static inline uint8_t i32toa(int32_t v, char *buf) {
  if (v < 0) { buf[0] = '-'; return 1 + u32toa((uint32_t)(-v), buf + 1); }
  return u32toa((uint32_t)v, buf);
}

static uint8_t buildDayLine(uint64_t elapsedUs, char *out) {
  out[0] = 'D'; out[1] = 'A'; out[2] = 'Y'; out[3] = ':'; out[4] = ' ';
  uint8_t pos = 5;
  const uint64_t USPERDAY = 86400000000ULL;
  uint32_t dayInt = (uint32_t)(elapsedUs / USPERDAY);
  uint64_t rem    = elapsedUs % USPERDAY;
  pos += u32toa(dayInt, out + pos);
  out[pos++] = '.';
  for (int i = 0; i < 12; i++) {
    rem *= 10ULL;
    uint32_t d = (uint32_t)(rem / USPERDAY);
    out[pos++] = (char)('0' + d);
    rem %= USPERDAY;
  }
  out[pos++] = '\n';
  return pos;
}

// Recompute cached values derived from settings.
static inline void cacheDerivedSettings() {
  rotateAfterMillis = settings.openNewLogFilesAfter * 1000UL;
  // Pre/post split: evPrePct% pre, (100-evPrePct)% post. (pct*10) converts the
  // window ms to us in one multiply, e.g. 20% of 4000 ms = 4000*200 = 800000 us.
  evPreWindowUs     = (uint32_t)settings.eventWindowMs * (uint32_t)(settings.evPrePct * 10u);
  evPostWindowUs    = (uint32_t)settings.eventWindowMs * (uint32_t)((100u - settings.evPrePct) * 10u);

  // Threshold in raw^2.  HP filter removes gravity, so this is just thresh^2.
  uint32_t rawPerG = 16384u >> (settings.imuAccFSS & 3);
  int32_t  devRaw  = (int32_t)(settings.eventThreshMg * (float)rawPerG / 1000.0f);
  if (devRaw > 32767) devRaw = 32767;
  evMagHigh2 = (uint32_t)devRaw * (uint32_t)devRaw;
}

static void enableCIPOpullUp() {
  am_hal_gpio_pincfg_t cfg = g_AM_BSP_GPIO_IOM0_MISO;
  cfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
  pin_config(PinName(PIN_SPI_CIPO), cfg);
}

static void applyBurstMode() {
  if (!burstInitDone) {
    am_hal_burst_avail_e avail;
    if (am_hal_burst_mode_initialize(&avail) == AM_HAL_STATUS_SUCCESS &&
        avail == AM_HAL_BURST_AVAIL) {
      burstAvailable = true;
    }
    burstInitDone = true;
  }
  if (!burstAvailable) {
    burstMode = false;
    Serial.println(F("BURST: not available on this chip - 48 MHz"));
    return;
  }
  am_hal_burst_mode_e mode;
  if (settings.burstEnabled) {
    if (am_hal_burst_mode_enable(&mode) == AM_HAL_STATUS_SUCCESS && mode == AM_HAL_BURST_MODE) {
      burstMode = true;  Serial.println(F("BURST: CPU at 96 MHz (TurboSPOT)"));
    } else { burstMode = false; Serial.println(F("BURST: enable failed - 48 MHz")); }
  } else {
    if (am_hal_burst_mode_disable(&mode) == AM_HAL_STATUS_SUCCESS && mode == AM_HAL_NORMAL_MODE) {
      burstMode = false; Serial.println(F("BURST: CPU at 48 MHz (normal, low power)"));
    } else { Serial.println(F("BURST: disable failed")); }
  }
}

static void stampCreate(FsFile *f) {
  myRTC.getTime();
  f->timestamp(T_CREATE, (uint16_t)(myRTC.year+2000), myRTC.month, myRTC.dayOfMonth,
               myRTC.hour, myRTC.minute, myRTC.seconds);
}
static void stampWrite(FsFile *f) {
  myRTC.getTime();
  f->timestamp(T_ACCESS|T_WRITE, (uint16_t)(myRTC.year+2000), myRTC.month, myRTC.dayOfMonth,
               myRTC.hour, myRTC.minute, myRTC.seconds);
}

static void setAccelScale() {
  static const int32_t mul[] = { 400000, 800000, 1600000, 3200000 };
  accelMulQ16 = mul[settings.imuAccFSS & 3];
}

static void selectBank0() {
  SPI.beginTransaction(ICM_SPI_SETTINGS);
  digitalWrite(PIN_IMU_CHIP_SELECT, LOW);
  SPI.transfer(ICM_REG_BANK_SEL);
  SPI.transfer(ICM_BANK0);
  digitalWrite(PIN_IMU_CHIP_SELECT, HIGH);
  SPI.endTransaction();
}

static inline void readAccelRaw(bool scale) {
  SPI.beginTransaction(ICM_SPI_SETTINGS);
  digitalWrite(PIN_IMU_CHIP_SELECT, LOW);
  SPI.transfer(ICM_REG_ACCEL_XOUT | 0x80);
  uint8_t b0 = SPI.transfer(0); uint8_t b1 = SPI.transfer(0);
  uint8_t b2 = SPI.transfer(0); uint8_t b3 = SPI.transfer(0);
  uint8_t b4 = SPI.transfer(0); uint8_t b5 = SPI.transfer(0);
  digitalWrite(PIN_IMU_CHIP_SELECT, HIGH);
  SPI.endTransaction();
  rawAx = (int16_t)((b0 << 8) | b1);
  rawAy = (int16_t)((b2 << 8) | b3);
  rawAz = (int16_t)((b4 << 8) | b5);
  // mg x100 scaling is needed ONLY for continuous-mode CSV lines; event mode
  // stores raw and scales at dump time, so skip these 3 multiplies when !scale.
  if (scale) {
    accelX100X = (int32_t)(((int64_t)rawAx * accelMulQ16) >> 16);
    accelX100Y = (int32_t)(((int64_t)rawAy * accelMulQ16) >> 16);
    accelX100Z = (int32_t)(((int64_t)rawAz * accelMulQ16) >> 16);
  }
}

static inline void readAccelLib(bool scale) {
  myICM.getAGMT();
  rawAx = myICM.agmt.acc.axes.x; rawAy = myICM.agmt.acc.axes.y; rawAz = myICM.agmt.acc.axes.z;
  if (scale) {
    accelX100X = (int32_t)(myICM.accX() * 100.0f);
    accelX100Y = (int32_t)(myICM.accY() * 100.0f);
    accelX100Z = (int32_t)(myICM.accZ() * 100.0f);
  }
}

static inline void readAccel(bool scale) { if (useRawRead) readAccelRaw(scale); else readAccelLib(scale); }

static inline bool imuDataReady() {
  if (useGpioReady) return digitalRead(PIN_IMU_INT) == HIGH;
  return myICM.dataReady();
}

// Format one continuous-mode CSV line directly into the caller's buffer (the SD
// batch buffer), returning its length - no intermediate copy. Max width 38 B.
static inline uint8_t buildLineInto(uint8_t *d, uint32_t captureMicros) {
  uint8_t pos = u32toa(captureMicros, (char *)d);
  d[pos++] = ','; pos += i32toa(accelX100X, (char *)d + pos);
  d[pos++] = ','; pos += i32toa(accelX100Y, (char *)d + pos);
  d[pos++] = ','; pos += i32toa(accelX100Z, (char *)d + pos);
  d[pos++] = '\n'; return pos;
}

static inline void sdFlushSectors(bool forceAll) {
  uint16_t whole = (sdBufLen / 512) * 512;
  if (forceAll && sdBufLen > whole) whole = sdBufLen;
  if (whole == 0) return;
  dataFile.write(sdBuf, whole);
  sectorsSinceSync += (whole + 511) / 512;
  uint16_t remain = sdBufLen - whole;
  if (remain > 0) memmove(sdBuf, sdBuf + whole, remain);
  sdBufLen = remain;
  if (sectorsSinceSync >= SYNC_EVERY_SECTORS) { dataFile.sync(); sectorsSinceSync = 0; }
}

// ===========================================================================
//  EVENT CAPTURE
// ===========================================================================
static inline void evAppend(const uint8_t *p, uint8_t n) {
  memcpy(sdBuf + sdBufLen, p, n); sdBufLen += n; fileBytesWritten += n;
  if (sdBufLen >= SD_WRITE_THRESHOLD) sdFlushSectors(false);
}

static inline void evStore(uint32_t m) {
  uint32_t d = m - evLastMicros;                              // us since previous stored sample
  // Start a CLEAN chunk on the first sample, or whenever the gap since the last
  // sample is too large for a uint16 delta (>~65 ms). Such a gap must never sit
  // mid-chunk - it would corrupt the offset column and the DAY anchor; forcing a
  // boundary here lands it only on a chunk's first sample, where the delta is
  // unused (the offset column starts at 0). The recording/close paths flush first
  // (below) so no captured data is lost; the idle pre-roll is simply restarted.
  if (evCapLen == 0 || d > 0xFFFFu) {
    evCapStart = evHead; evCapLen = 0; evStartMicros = m; d = 0;
  }
  evDelta[evHead] = (uint16_t)d;
  evAx[evHead] = rawAx; evAy[evHead] = rawAy; evAz[evHead] = rawAz;
  evLastMicros = m;
  if (++evHead >= EVENT_RING_SAMPLES) evHead = 0; evCapLen++;
}

// Dump [evCapStart .. evHead) to SD: a DAY marker (absolute decimal day of the
// FIRST sample, 12 dp) is the t=0 reference, then CSV lines whose first column
// is a 16-bit microsecond offset from that marker - it starts at 0 and rolls
// over at 65536 through the window - then a sync. Each chunk (including each
// mid-event ring-full flush) gets its own DAY marker and restarts the offset at
// 0. Caller resets evCapStart/evCapLen afterwards.
static void evDumpChunk() {
  if (evCapLen == 0) return;
  uint32_t F = evStartMicros;                 // absolute micros of the first sample
  uint32_t ageUs = (uint32_t)(prevMicros - F);
  uint64_t nowRaw = ((uint64_t)usHigh << 32) | (uint64_t)prevMicros;
  uint64_t elapsedNow = nowRaw - (uint64_t)bootMicros;
  uint64_t firstElapsed = elapsedNow - (uint64_t)ageUs;
  char dbuf[32]; uint8_t dlen = buildDayLine(firstElapsed, dbuf);
  evAppend((const uint8_t *)dbuf, dlen);
  uint32_t idx = evCapStart;
  uint16_t off = 0;                                  // 16-bit us offset from the DAY marker (=0)
  for (uint32_t k = 0; k < evCapLen; k++) {
    if (k > 0) { if (++idx >= EVENT_RING_SAMPLES) idx = 0; off += evDelta[idx]; }  // rolls at 65536
    int32_t sx = (int32_t)(((int64_t)evAx[idx] * accelMulQ16) >> 16);
    int32_t sy = (int32_t)(((int64_t)evAy[idx] * accelMulQ16) >> 16);
    int32_t sz = (int32_t)(((int64_t)evAz[idx] * accelMulQ16) >> 16);
    char lb[48]; uint8_t pos = u32toa(off, lb);
    lb[pos++] = ','; pos += i32toa(sx, lb + pos);
    lb[pos++] = ','; pos += i32toa(sy, lb + pos);
    lb[pos++] = ','; pos += i32toa(sz, lb + pos);
    lb[pos++] = '\n'; evAppend((const uint8_t *)lb, pos);
  }
  sdFlushSectors(true); dataFile.sync(); stampWrite(&dataFile);
  if (settings.ledMode == 0) pwrLedOn();
}

// One event-mode step for the current sample (rawAx/Ay/Az at micros M).
//
// Two independent state layers:
//   1. selfActive  - our OWN threshold window (pre/post split). It alone
//                    drives TRIG_OUT HIGH (active high), in AUTO mode only.
//                    Being a pure function of our own accel + own post-window
//                    timer (never "am I recording"), it cannot latch a partner
//                    on, so two units can't hold each other recording forever.
//   2. recording   - selfActive OR partner-HIGH. The window written to SD is the
//                    UNION: it opens when EITHER goes HIGH and closes only when
//                    BOTH are LOW. The rolling pre-history ring supplies the
//                    pre-roll for whichever source fires first.
//
// Ring-full mid-event: flush the whole buffer to SD (its own DAY marker) and
// keep capturing in the SAME window with TRIG_OUT still HIGH, repeating as long
// as the event lasts (so long events span several DAY-marked chunks, each up to
// the ring's worth, rather than ever overflowing RAM).
static void eventModeStep(uint32_t M) {
  const bool pAuto = partnerAuto();   // cached: constant within this step
  // HP filter: slow LP tracks gravity, subtract for a dynamic-only trigger.
  // LP state is Q16 (raw << 16). Only the trigger sees this; logged data is raw.
  hpLpAx += HP_ALPHA_Q16 * ((int32_t)rawAx - (hpLpAx >> 16));
  hpLpAy += HP_ALPHA_Q16 * ((int32_t)rawAy - (hpLpAy >> 16));
  hpLpAz += HP_ALPHA_Q16 * ((int32_t)rawAz - (hpLpAz >> 16));
  int16_t hpAx = rawAx - (int16_t)(hpLpAx >> 16);
  int16_t hpAy = rawAy - (int16_t)(hpLpAy >> 16);
  int16_t hpAz = rawAz - (int16_t)(hpLpAz >> 16);
  uint32_t mag2 = (uint32_t)((int32_t)hpAx * hpAx)
                + (uint32_t)((int32_t)hpAy * hpAy)
                + (uint32_t)((int32_t)hpAz * hpAz);
  bool over = (mag2 > evMagHigh2);              // no low-side test; HP zeroed gravity

  // HPF settling: for the first HPF_SETTLE_MS after logging starts, keep updating
  // the filter (above) but DON'T let the threshold fire - gravity is still being
  // removed. Costs one branch once settled (no millis() call after that). Partner
  // triggering is unaffected.
  if (hpfSettling) {
    if ((uint32_t)(millis() - logStartMillis) >= HPF_SETTLE_MS) hpfSettling = false;
    else over = false;
  }

  // Post-handshake hold: after a partner pairing, suppress ALL event starts (own
  // threshold AND the partner line) for PARTNER_HOLD_MS so the two units settle in
  // sync before capturing. Costs a millis() only while a hold is active (rare).
  bool inHold = (partnerHoldUntil != 0) && ((int32_t)(millis() - partnerHoldUntil) < 0);
  if (inHold) over = false;
  else if (partnerHoldUntil != 0) {                          // hold just expired -> one-shot "logging live" flash
    partnerHoldUntil = 0;
    holdEndFlashRunning = true; holdEndFlashStart = millis();
    holdEndFlashToggle = holdEndFlashStart; holdEndFlashState = false;
  }

  // A sampling gap too large for a uint16 delta to hold (>~65 ms: a mid-event SD
  // flush, a file rotation, the rare 1/5-rate glitch, or an IMU stall) must never
  // be delta-encoded. Detect it and force a clean chunk boundary: while recording
  // we flush what we have first (no data lost) then continue in a fresh chunk;
  // while idle, evStore simply restarts the pre-roll.
  bool bigGap = (evCapLen > 0) && ((uint32_t)(M - evLastMicros) > 0xFFFFu);

  // --- Own-event state -> TRIG_OUT (active high, AUTO mode only) ------------
  if (over) {
    selfPostUntil = M + evPostWindowUs;                       // (re)arm post-window timer
    if (!selfActive) { selfActive = true; if (pAuto) trigOutHigh(); }
  } else if (selfActive && (int32_t)(M - selfPostUntil) >= 0) {
    selfActive = false; if (pAuto) trigOutLow();              // own window ended
  }

  // --- Partner input (active high) - honoured only in AUTO mode ------------
  bool partnerActive = pAuto && partnerLineHigh && !inHold && (partnerPulseCount == 0);  // a pairing TOGGLE sets partnerPulseCount; never let it open a partner window

  // --- Record while EITHER our own event OR the partner's is live ----------
  bool recordingNow = selfActive || partnerActive;

  if (recordingNow) {
    if (!recordingPrev) evPartnerInWindow = false;            // new window opens
    if (partnerActive)  evPartnerInWindow = true;             // mark partner involvement
    // Ring full mid-event, OR a sampling gap too big for a uint16 delta (bigGap):
    // drop our trig-out LOW while the (blocking) SD write runs - a following
    // partner sees the window boundary - flush what we have (own DAY marker), then
    // raise trig-out again as the window continues in a fresh chunk.
    if (evCapLen >= EVENT_RING_SAMPLES || bigGap) {
      bool drove = selfActive && pAuto;                       // were WE driving the line?
      if (drove) trigOutLow();
      evDumpChunk();
      evCapStart = evHead; evCapLen = 0;
      // After flush: re-check threshold (using 'over' from this sample, already
      // computed before the flush). If the shock has subsided, end our own event
      // immediately rather than extending into another full ring fill. Partner
      // input is still honoured independently via partnerActive.
      if (!over) {
        selfActive = false;                                   // our event ends now
        // trig-out already LOW; let the close path fire on the next sample
      } else {
        if (drove) trigOutHigh();                             // shock persists: resume signalling
      }
    }
    evStore(M);
  } else if (recordingPrev) {
    // Window just closed (own post-window elapsed AND partner LOW): final flush.
    // Our trig-out is already LOW here (selfActive went false), so no line drop.
    if (evCapLen >= EVENT_RING_SAMPLES || bigGap) { evDumpChunk(); evCapStart = evHead; evCapLen = 0; }
    evStore(M);
    evDumpChunk();
    evCapStart = evHead; evCapLen = 0;                        // re-arm, fresh pre-history
    // LED on a true window close:
    if (hsFlashArmed && evPartnerInWindow) {
      // First live partner event with no startup handshake -> 1 s blue blink.
      hsFlashArmed = false; hsFlashRunning = true;
      hsFlashStart = millis(); hsFlashToggle = hsFlashStart; hsFlashState = false;
    } else if (((settings.ledMode == 2) || (settings.ledMode == 0)) && !hsFlashRunning && !holdEndFlashRunning) {
      // After-event short flash (turned off by serviceLeds after AFTER_FLASH_MS).
      statLedOn(); afterFlashOn = true; afterFlashStart = millis();
    }
  } else {
    // ARMED: keep a rolling pre-window of history for the next trigger. Hard-cap
    // to the ring so the pre-roll can never overflow regardless of window size.
    evStore(M);
    while (evCapLen > 1 && ((uint32_t)(M - evStartMicros) > evPreWindowUs
                            || evCapLen >= EVENT_RING_SAMPLES)) {
      if (++evCapStart >= EVENT_RING_SAMPLES) evCapStart = 0;
      evStartMicros += evDelta[evCapStart];           // advance anchor to new oldest sample
      evCapLen--;
    }
  }
  recordingPrev = recordingNow;

  // Rotate files only between events (never split a capture across files).
  // Time-based rotation skips the rotate if the current file has no data beyond
  // the header (fileBytesWritten <= 16) to avoid creating empty files during
  // long quiet periods. Size-based always implies data, so no guard needed.
  if (!recordingNow) {
    uint32_t now = millis();
    bool rotT = (rotateAfterMillis > 0) && (now - lastRotateMillis >= rotateAfterMillis);
    bool rotS = (settings.maxLogFileBytes > 0) && (fileBytesWritten >= settings.maxLogFileBytes);
    if (rotT && fileBytesWritten <= 16) {
      lastRotateMillis = now;                                 // reset timer; don't create empty file
    } else if (rotT || rotS) {
      rotateLogFile();
    }
  }

  // Heartbeat LED (low-power mode 1).
  if (settings.ledMode == 1) {
    uint32_t now = millis();
    static bool eLedOn = false; static uint32_t eLedStart = 0;
    if (!eLedOn && (now - lastLedFlashMillis >= 15000UL)) {
      lastLedFlashMillis = now; eLedStart = now; eLedOn = true; statLedOn();
    }
    if (eLedOn && (now - eLedStart >= 2)) { statLedOff(); eLedOn = false; }
  }
}

// -- parseLine ----------------------------------------------------------------
static void parseLine(char *str) {
  char *eq = strchr(str, '='); if (!eq) return;
  *eq = '\0';
  char *key = str; while (*key==' '||*key=='\t') key++;
  char *val = eq+1; while (*val==' '||*val=='\t') val++;
  char *end = val + strlen(val) - 1;
  while (end >= val && (*end=='\r'||*end=='\n'||*end==' ')) *end-- = '\0';
  if (*val == '\0') return;
  double d = strtod(val, nullptr);

  if      (!strcmp(key,"sizeOfSettings"))       { }
  else if (!strcmp(key,"olaIdentifier"))         settings.olaIdentifier        = (int)d;
  else if (!strcmp(key,"nextLogNumber"))         settings.nextLogNumber         = (int)d;
  else if (!strcmp(key,"usBetweenReadings"))     settings.usBetweenReadings     = (uint32_t)d;
  else if (!strcmp(key,"enableTerminalOutput"))  settings.enableTerminalOutput   = (bool)d;
  else if (!strcmp(key,"imuAccDLPF"))            settings.imuAccDLPF             = (bool)d;
  else if (!strcmp(key,"imuAccFSS"))             settings.imuAccFSS              = (int)d;
  else if (!strcmp(key,"imuAccDLPFBW"))          settings.imuAccDLPFBW           = (int)d;
  else if (!strcmp(key,"recordEventsOnly"))      settings.recordEventsOnly       = (bool)d;
  else if (!strcmp(key,"eventWindowMs"))         settings.eventWindowMs          = (uint32_t)d;
  else if (!strcmp(key,"eventThreshMg"))         settings.eventThreshMg          = (float)d;
  else if (!strcmp(key,"evPrePct"))              settings.evPrePct               = (uint8_t)d;
  else if (!strcmp(key,"openNewLogFilesAfter"))  settings.openNewLogFilesAfter   = (uint32_t)d;
  else if (!strcmp(key,"maxLogFileBytes"))       settings.maxLogFileBytes        = (uint32_t)d;
  else if (!strcmp(key,"burstEnabled"))          settings.burstEnabled           = (bool)d;
  else if (!strcmp(key,"terminalBaudRate"))      settings.terminalBaudRate       = (int)d;
  else if (!strcmp(key,"ledMode"))               settings.ledMode                = (uint8_t)d;
  else if (!strcmp(key,"partnerMode"))           settings.partnerMode            = (uint8_t)d;
}

static void writeSettingsFile() {
  if (!sdOnline) return;
  sd.remove("OLA_settings.txt");
  FsFile f;
  if (!f.open("OLA_settings.txt", O_CREAT | O_WRITE)) return;
  f.print(F("sizeOfSettings="));       f.println(settings.sizeOfSettings);
  f.print(F("olaIdentifier="));        f.println(settings.olaIdentifier);
  f.print(F("nextLogNumber="));        f.println(settings.nextLogNumber);
  f.print(F("usBetweenReadings="));    f.println(settings.usBetweenReadings);
  f.print(F("enableTerminalOutput=")); f.println((int)settings.enableTerminalOutput);
  f.print(F("imuAccDLPF="));           f.println((int)settings.imuAccDLPF);
  f.print(F("imuAccFSS="));            f.println(settings.imuAccFSS);
  f.print(F("imuAccDLPFBW="));         f.println(settings.imuAccDLPFBW);
  f.print(F("recordEventsOnly="));     f.println((int)settings.recordEventsOnly);
  f.print(F("eventWindowMs="));        f.println(settings.eventWindowMs);
  f.print(F("eventThreshMg="));        f.println(settings.eventThreshMg, 1);
  f.print(F("evPrePct="));             f.println(settings.evPrePct);
  f.print(F("openNewLogFilesAfter=")); f.println(settings.openNewLogFilesAfter);
  f.print(F("maxLogFileBytes="));      f.println(settings.maxLogFileBytes);
  f.print(F("burstEnabled="));         f.println((int)settings.burstEnabled);
  f.print(F("terminalBaudRate="));     f.println(settings.terminalBaudRate);
  f.print(F("ledMode="));              f.println(settings.ledMode);
  f.print(F("partnerMode="));          f.println(settings.partnerMode);
  stampWrite(&f); f.close();
}

void recordSettings() {
  settings.sizeOfSettings = sizeof(settings);
  EEPROM.put(0, settings);
  EEPROM.put(BUILD_STAMP_ADDR, BUILD_STAMP);  // keep stamp in sync
  writeSettingsFile();
}

static void clampSettings() {
  // File rotation: enforce minimums (0 = disabled, else >= min)
  if (settings.openNewLogFilesAfter > MAX_LOGFILE_SECONDS_CAP) settings.openNewLogFilesAfter = MAX_LOGFILE_SECONDS_CAP;
  if (settings.openNewLogFilesAfter > 0 && settings.openNewLogFilesAfter < MIN_FILE_SECONDS)
      settings.openNewLogFilesAfter = MIN_FILE_SECONDS;
  if (settings.maxLogFileBytes > MAX_LOGFILE_BYTES_CAP) settings.maxLogFileBytes = MAX_LOGFILE_BYTES_CAP;
  if (settings.maxLogFileBytes > 0 && settings.maxLogFileBytes < MIN_FILE_BYTES)
      settings.maxLogFileBytes = MIN_FILE_BYTES;
  // Event window
  if (settings.eventWindowMs < MIN_WINDOW_MS)          settings.eventWindowMs = MIN_WINDOW_MS;
  uint32_t ringMs = (uint32_t)(estimateRingSeconds() * 1000.0f);
  if (ringMs < MIN_WINDOW_MS) ringMs = MIN_WINDOW_MS;
  if (settings.eventWindowMs > ringMs)                 settings.eventWindowMs = ringMs;
  // Threshold
  if (settings.eventThreshMg < MIN_THRESH_MG)          settings.eventThreshMg = MIN_THRESH_MG;
  if (settings.eventThreshMg > MAX_THRESH_MG)          settings.eventThreshMg = MAX_THRESH_MG;
  // Pre/post split
  if (settings.evPrePct < 2)                           settings.evPrePct = 2;
  if (settings.evPrePct > 98)                          settings.evPrePct = 98;
  // Event mode rate floor: 20 Hz minimum (for 16-bit delta safety)
  if (settings.recordEventsOnly && settings.usBetweenReadings > 0) {
    uint32_t minUs = 1000000UL / MIN_EVENT_RATE_HZ;   // 50000 us = 20 Hz
    if (settings.usBetweenReadings > minUs) settings.usBetweenReadings = minUs;
  }
  if (settings.partnerMode > 1)                        settings.partnerMode = 1;
  if (settings.ledMode > 3)                            settings.ledMode = 2;
}

static void loadSettings() {
  // --- Auto-detect fresh flash: compare compile timestamp in EEPROM. --------
  // __DATE__ " " __TIME__ changes every compile, so every flash is detected
  // automatically — no manual identifier bumps needed.
  char storedStamp[BUILD_STAMP_LEN];
  EEPROM.get(BUILD_STAMP_ADDR, storedStamp);
  if (memcmp(storedStamp, BUILD_STAMP, BUILD_STAMP_LEN) != 0) {
    Serial.println(F("New firmware detected - applying compiled defaults to EEPROM + SD"));
    clampSettings();
    cacheDerivedSettings();
    recordSettings();
    EEPROM.put(BUILD_STAMP_ADDR, BUILD_STAMP);
    return;
  }
  // --- Normal boot: load saved settings from EEPROM, overlay SD file. -------
  uint32_t probe = 0; EEPROM.get(0, probe);
  if (probe == 0xFFFFFFFF) { Serial.println(F("EEPROM blank")); recordSettings(); return; }
  int sz = 0; EEPROM.get(0, sz);
  if (sz != (int)sizeof(settings)) { Serial.println(F("Settings size changed")); recordSettings(); return; }
  EEPROM.get(0, settings);
  if (sdOnline && sd.exists("OLA_settings.txt")) {
    FsFile f;
    if (f.open("OLA_settings.txt", O_READ)) {
      char line[80]; while (f.available()) { int n = f.fgets(line, sizeof(line)); if (n>0) parseLine(line); }
      f.close(); Serial.println(F("Settings loaded from SD..."));
    }
  }
  clampSettings();
  cacheDerivedSettings();
  recordSettings();
}

static void beginSD() {
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pin_config(PinName(PIN_MICROSD_POWER), g_AM_HAL_GPIO_OUTPUT);
  pinMode(PIN_MICROSD_CHIP_SELECT, OUTPUT);
  pin_config(PinName(PIN_MICROSD_CHIP_SELECT), g_AM_HAL_GPIO_OUTPUT);
  digitalWrite(PIN_MICROSD_CHIP_SELECT, HIGH);
  delay(5); sdPowerOn();
  for (int i = 0; i < 100; i++) delay(1);
  if (!sd.begin(SD_CONFIG)) {
    Serial.println(F("SD init failed - retry")); for (int i=0;i<250;i++) delay(1);
    if (!sd.begin(SD_CONFIG)) { Serial.println(F("SD FAIL. Halting.")); while(1){statLedOn();delay(200);statLedOff();delay(200);} }
  }
  if (!sd.chdir()) { Serial.println(F("SD chdir fail")); while(1){statLedOn();delay(100);statLedOff();delay(100);} }
  sdOnline = true;
  Serial.println(F("SD online..."));
}

static void openNextLogFile(bool reuseEmpty) {
  // Find the first FREE slot (nextLogNumber is a persisted hint of where to look).
  if (settings.nextLogNumber > 0) settings.nextLogNumber--;
  char name[30];
  while (true) {
    snprintf(name, sizeof(name), "dataLog%05d.TXT", settings.nextLogNumber);
    if (!sd.exists(name)) break;
    settings.nextLogNumber++;
    if (settings.nextLogNumber >= 100000) { settings.nextLogNumber = 99999; break; }
  }
  // At boot (reuseEmpty) the highest existing file is the slot just below the free
  // one: if it never recorded data (header-only, <=16 B), reuse that number so a
  // reboot - or a restart from the startup rate check - that logged nothing does
  // not waste a fresh file. Rotation passes reuseEmpty=false (always a new file).
  uint32_t useNumber = settings.nextLogNumber;
  if (reuseEmpty && settings.nextLogNumber > 0) {
    char pname[30];
    snprintf(pname, sizeof(pname), "dataLog%05d.TXT", settings.nextLogNumber - 1);
    FsFile pf;
    if (pf.open(pname, O_RDONLY)) {
      uint64_t sz = pf.fileSize(); pf.close();
      if (sz <= 16) useNumber = settings.nextLogNumber - 1;
    }
  }
  snprintf(name, sizeof(name), "dataLog%05d.TXT", useNumber);
  strncpy(dataFileName, name, sizeof(dataFileName)-1);
  dataFileName[sizeof(dataFileName)-1] = '\0';
  settings.nextLogNumber = useNumber + 1;                     // next rotation continues upward
  if (!dataFile.open(dataFileName, O_CREAT|O_TRUNC|O_WRITE)) {
    Serial.print(F("ERR open ")); Serial.println(dataFileName);
    while(1){statLedOn();delay(500);statLedOff();delay(500);}
  }
  stampCreate(&dataFile);
  sdBufLen = 0; sectorsSinceSync = 0;
  const char* hdr = "micros,aX,aY,aZ\n";
  dataFile.write(hdr, 16); dataFile.sync();
  fileBytesWritten = 16;
  recordSettings();
}

static void rotateLogFile() {
  sdFlushSectors(true);
  dataFile.sync(); stampWrite(&dataFile); dataFile.close();
  dataFileName[0] = '\0';
  openNextLogFile(false);
  lastRotateMillis = millis();
}

// ===========================================================================
//  PARTNER TOGGLE-HANDSHAKE  (pairing + DAY-clock sync)
// ===========================================================================
// The handshake replaces the old "hold TRIG_OUT HIGH at boot" announce, which a
// partner already logging (steady HIGH) could mistake for an event. Instead a
// connecting unit TOGGLES TRIG_OUT (250 us on/off); the other unit recognises the
// brief HIGH->LOW pulses (the ISR sets partnerHsPulse) and toggles back. Roles:
//   INITIATOR (just-booted or just-connected unit): toggles, stops the instant it
//     sees the partner toggling back (>= HS_PULSES_CONFIRM pulses).
//   RESPONDER (the unit that detected the pulses): toggles back, then stops when
//     the initiator goes quiet (no edge for HS_STOP_QUIET_US).
// The initiator zeroes its DAY clock when it stops; the responder zeroes its clock
// and adds 1 ms to cancel the 1 ms it waits to confirm quiet, keeping both aligned.

// Re-zero the DAY clock so it reads 0 at 'completeRaw' (64-bit micros of handshake
// completion). The responder passes addMs=true (the +1 ms sync correction).
static inline void partnerResetDayClock(uint64_t completeRaw, bool addMs) {
  bootMicros = addMs ? (completeRaw - 1000ULL) : completeRaw;
}

// Create an EMPTY marker file named for the current log + the connect day, e.g.
// dataLog00005_partner_connected_at_day_10_123456789012.TXT (exFAT long name).
static void makePartnerMarkerFile(uint64_t elapsedUs) {
  char nm[80];
  uint32_t fn = (settings.nextLogNumber > 0) ? (settings.nextLogNumber - 1) : 0;
  int pos = snprintf(nm, sizeof(nm), "dataLog%05lu_partner_connected_at_day_", (unsigned long)fn);
  const uint64_t USPERDAY = 86400000000ULL;
  uint32_t dayInt = (uint32_t)(elapsedUs / USPERDAY);
  uint64_t rem    = elapsedUs % USPERDAY;
  pos += snprintf(nm + pos, sizeof(nm) - pos, "%lu_", (unsigned long)dayInt);
  for (int i = 0; i < 12 && pos < (int)sizeof(nm) - 5; i++) {
    rem *= 10ULL; nm[pos++] = (char)('0' + (uint32_t)(rem / USPERDAY)); rem %= USPERDAY;
  }
  snprintf(nm + pos, sizeof(nm) - pos, ".TXT");
  FsFile mf;
  if (mf.open(nm, O_CREAT | O_TRUNC | O_WRITE)) { mf.sync(); mf.close(); }
}

// RESPONDER side (runtime). Toggle back at HS_TOGGLE_US and watch for the partner
// to go quiet (it stops once it has seen us). Returns the 64-bit micros at which we
// stopped (handshake complete) or 0 on timeout. Requires having actually SEEN the
// partner toggling (>= HS_PULSES_CONFIRM) so a lone glitch can't fake a pairing.
// Semi-blocking on purpose: logging is intentionally paused to pair.
static uint64_t hsRespond() {
  uint32_t t0 = micros(), tNx = t0; bool out = false;
  partnerPulseCount = 0;                                    // count the partner's pulses from now
  while ((uint32_t)(micros() - t0) < HS_RESP_TIMEOUT_US) {
    uint32_t nowUs = micros();
    if ((int32_t)(nowUs - tNx) >= 0) { out = !out; out ? trigOutHigh() : trigOutLow(); tNx += HS_TOGGLE_US; }
    if ((uint32_t)(nowUs - t0) > HS_MIN_RESP_US && partnerPulseCount >= HS_PULSES_CONFIRM) {
      if ((uint32_t)(nowUs - partnerEdgeMicros) > HS_STOP_QUIET_US) {   // partner stopped
        trigOutLow();
        uint32_t cUs = micros(); if (cUs < prevMicros) usHigh++; prevMicros = cUs;
        return ((uint64_t)usHigh << 32) | (uint64_t)cUs;
      }
    }
  }
  trigOutLow();
  return 0;
}

// One INITIATOR toggle step (startup). Toggles at HS_TOGGLE_US; returns true (with
// *completeRaw set to the stop time) the instant the responder is seen toggling.
static bool hsInitiatorStep(uint32_t *tNx, bool *out, uint64_t *completeRaw) {
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - *tNx) >= 0) { *out = !*out; *out ? trigOutHigh() : trigOutLow(); *tNx += HS_TOGGLE_US; }
  if (partnerPulseCount >= HS_PULSES_CONFIRM) {
    trigOutLow();
    uint32_t cUs = micros(); if (cUs < prevMicros) usHigh++; prevMicros = cUs;
    *completeRaw = ((uint64_t)usHigh << 32) | (uint64_t)cUs;
    return true;
  }
  return false;
}

// Runtime partner-connect handler, called from loop() when the ISR flags a pairing
// pulse. Finishes/abandons the current capture, runs the responder handshake, (if
// the DAY clock has advanced past DAY_RESET_THRESH_US) splits the log file with an
// empty marker, re-zeroes the clock (+1 ms), blinks, and holds events 5 s. Semi-
// blocking by design - we deliberately pause logging to pair.
static void handlePartnerConnect() {
  partnerHsPulse = false;

  // Current day on the OLD clock (for the marker name + the split decision).
  uint32_t nUs = micros(); if (nUs < prevMicros) usHigh++; prevMicros = nUs;
  uint64_t elapsedOld = (((uint64_t)usHigh << 32) | (uint64_t)nUs) - bootMicros;

  // Finish a REAL own event (flush, keep its data); discard a buffer that was only
  // partner-driven (the toggling's own brief HIGHs) - never write that tiny "event".
  if (selfActive && evCapLen > 0) evDumpChunk();
  evCapStart = evHead; evCapLen = 0;
  selfActive = false; recordingPrev = false; evPartnerInWindow = false;
  trigOutLow();

  uint64_t completeRaw = hsRespond();
  if (completeRaw == 0) {                       // no clean handshake (glitch/partner gone): resume
    partnerLineHigh = (digitalRead(PIN_TRIG_IN) == HIGH);
    partnerPulseCount = 0; partnerHsPulse = false;
    return;
  }

  // Paired. If the clock has advanced, split: an empty marker named for the connect
  // day, then a fresh data file with header for what follows.
  if (elapsedOld >= DAY_RESET_THRESH_US) {
    sdFlushSectors(true); dataFile.sync(); stampWrite(&dataFile); dataFile.close();
    makePartnerMarkerFile(elapsedOld);
    dataFileName[0] = '\0';
    openNextLogFile(false);
    lastRotateMillis = millis();
  }

  partnerResetDayClock(completeRaw, true);      // responder: clock 0 at handshake, +1 ms sync
  partnerPaired = true;
  partnerPulseCount = 0; partnerHsPulse = false; partnerLineHigh = false;
  armPartnerHold(completeRaw);                                    // hold ends 5 s after the handshake instant, not after this code
  holdEndFlashRunning = false;                                     // fresh hold; cancel any stale hold-end flash
  hsFlashArmed = false;                                            // link is confirmed by the blink below - don't also blue-blink on the first later event
  hpfSettling = true; logStartMillis = millis();
  blinkLed(PIN_STAT_LED, HS_BLINK_TOGGLE_MS, HS_BLINK_TOTAL_MS);   // synchronized pairing-confirm flash
}

static void configIMUInterrupt() {
  myICM.cfgIntActiveLow(false);
  myICM.cfgIntOpenDrain(false);
  myICM.cfgIntLatch(true);
  myICM.cfgIntAnyReadToClear(true);
  myICM.intEnableRawDataReady(true);
}

static void beginIMU() {
  pinMode(PIN_IMU_POWER, OUTPUT);
  pin_config(PinName(PIN_IMU_POWER), g_AM_HAL_GPIO_OUTPUT);
  pinMode(PIN_IMU_CHIP_SELECT, OUTPUT);
  digitalWrite(PIN_IMU_CHIP_SELECT, HIGH);
  pinMode(PIN_IMU_INT, INPUT);
  const int MAX_ATTEMPTS = 3;
  const uint16_t settleMs[] = {200, 500, 1000};
  for (int att = 0; att < MAX_ATTEMPTS; att++) {
    if (att > 0) { Serial.print(F("IMU attempt ")); Serial.println(att+1); }
    imuPowerOff(); for(int i=0;i<50;i++) delay(1);
    imuPowerOn();  for(uint16_t i=0;i<settleMs[att];i++) delay(1);
    myICM.begin(PIN_IMU_CHIP_SELECT, SPI, ICM_SPI_HZ);
    if (myICM.status == ICM_20948_Stat_Ok) break;
    if (att == MAX_ATTEMPTS-1) {
      Serial.println(F("IMU FAIL. Halting.")); while(1){statLedOn();delay(300);statLedOff();delay(300);}
    }
  }
  for(int i=0;i<25;i++) delay(1);
  myICM.startupDefault(false);
  myICM.enableDLPF(ICM_20948_Internal_Acc, settings.imuAccDLPF);
  ICM_20948_dlpcfg_t dlp; dlp.a = settings.imuAccDLPFBW; dlp.g = 0;
  myICM.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlp);
  ICM_20948_fss_t fss; fss.a = settings.imuAccFSS; fss.g = 0;
  myICM.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);
  ICM_20948_smplrt_t smplrt; smplrt.a = 0; smplrt.g = 0;
  myICM.setSampleRate(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, smplrt);
  setAccelScale(); configIMUInterrupt(); selectBank0();
  for(int i=0;i<50;i++) delay(1);

  useRawRead = false;
  readAccelLib(true);
  int32_t gx = accelX100X, gy = accelX100Y, gz = accelX100Z;
  selectBank0(); for(int i=0;i<5;i++) delay(1);
  readAccelRaw(true);
  int32_t rx = accelX100X, ry = accelX100Y, rz = accelX100Z;
  int32_t dx = rx>gx?rx-gx:gx-rx, dy = ry>gy?ry-gy:gy-ry, dz = rz>gz?rz-gz:gz-rz;
  if (dx < 5000 && dy < 5000 && dz < 5000) {
    useRawRead = true; Serial.println(F("ACCEL READ: raw fast path VERIFIED"));
  } else {
    useRawRead = false; Serial.println(F("ACCEL READ: raw path mismatch - fallback to getAGMT()"));
    Serial.print(F("  lib=")); Serial.print(gx); Serial.print(','); Serial.print(gy); Serial.print(','); Serial.print(gz);
    Serial.print(F("  raw=")); Serial.print(rx); Serial.print(','); Serial.print(ry); Serial.print(','); Serial.println(rz);
  }

  useGpioReady = false; int gpioHits = 0; readAccel(true);
  for (int cyc = 0; cyc < 5; cyc++) {
    delay(2); int pin = digitalRead(PIN_IMU_INT); bool libReady = myICM.dataReady();
    if (pin == HIGH && libReady) gpioHits++;
    else { Serial.print(F("GPIO INT cycle ")); Serial.print(cyc);
           Serial.print(F(" MISMATCH pin=")); Serial.print(pin);
           Serial.print(F(" lib=")); Serial.println(libReady?1:0); }
  }
  if (gpioHits == 5) { useGpioReady = true; Serial.println(F("READY POLL: GPIO INT pin VERIFIED")); }
  else { Serial.print(F("READY POLL: GPIO INT failed (")); Serial.print(gpioHits); Serial.println(F("/5) - fallback to SPI")); }
  readAccel(true);
  imuOnline = true; Serial.println(F("IMU online..."));
  Serial.print(F("First sample (mg x100): "));
  Serial.print(accelX100X); Serial.print(','); Serial.print(accelX100Y); Serial.print(','); Serial.println(accelX100Z);
}

static void emergencyFlush() {
  if (dataFile) {
    if (settings.recordEventsOnly && evCapLen > 0) evDumpChunk();
    sdFlushSectors(true); dataFile.sync(); stampWrite(&dataFile); dataFile.close();
  }
  while (1);
}

// ===========================================================================
//  MENU
// ===========================================================================
static void menuLedsOn()  { statLedOn();  pwrLedOn();  }
static void menuLedsOff() { statLedOff(); pwrLedOff(); }
static void menuBlink()   { menuLedsOff(); delay(100); menuLedsOn(); }
static void drainSerial() { delay(20); while (Serial.available()) { Serial.read(); delay(1); } }

static bool readNumber(uint32_t *result) {
  delay(10);
  while (Serial.available()) { char p = Serial.peek(); if (p=='\r'||p=='\n') Serial.read(); else break; }
  uint32_t val = 0; bool has = false; uint32_t ts = millis();
  while ((millis()-ts)/1000 < MENU_TIMEOUT_SEC) {
    if (!Serial.available()) continue;
    char c = Serial.read(); ts = millis(); menuBlink();
    if (c=='x'||c=='X') { Serial.println(F("  [cancelled]")); return false; }
    if (c=='\r'||c=='\n') {
      delay(5); while(Serial.available()){char p=Serial.peek();if(p=='\r'||p=='\n')Serial.read();else break;}
      if (has) { *result = val; Serial.println(); return true; }
      Serial.println(F("  [empty - cancelled]")); return false;
    }
    if (isDigit(c)) { Serial.write(c); val = val*10 + (uint32_t)(c-'0'); has = true; }
  }
  Serial.println(F("  [timeout]")); return false;
}

// Read a decimal float from serial (e.g. "3.5" or "0.25").
static bool readFloat(double *result) {
  delay(10);
  while (Serial.available()) { char p = Serial.peek(); if (p=='\r'||p=='\n') Serial.read(); else break; }
  char buf[24]; uint8_t len = 0; uint32_t ts = millis();
  while ((millis()-ts)/1000 < MENU_TIMEOUT_SEC) {
    if (!Serial.available()) continue;
    char c = Serial.read(); ts = millis(); menuBlink();
    if (c=='x'||c=='X') { Serial.println(F("  [cancelled]")); return false; }
    if (c=='\r'||c=='\n') {
      delay(5); while(Serial.available()){char p=Serial.peek();if(p=='\r'||p=='\n')Serial.read();else break;}
      if (len > 0) { buf[len] = '\0'; *result = strtod(buf, nullptr); Serial.println(); return true; }
      Serial.println(F("  [empty - cancelled]")); return false;
    }
    if ((isDigit(c) || c=='.' || c=='-') && len < 22) { Serial.write(c); buf[len++] = c; }
  }
  Serial.println(F("  [timeout]")); return false;
}

static bool readChar(char *result) {
  delay(10);
  while (Serial.available()) { char p = Serial.peek(); if (p=='\r'||p=='\n') Serial.read(); else break; }
  uint32_t ts = millis();
  while ((millis()-ts)/1000 < MENU_TIMEOUT_SEC) {
    if (!Serial.available()) continue;
    char c = Serial.read(); menuBlink();
    if (c=='\r'||c=='\n') continue;
    if (c=='x'||c=='X') { Serial.println(F("  [cancelled]")); return false; }
    *result = (c >= 'A' && c <= 'Z') ? (c + ('a' - 'A')) : c;
    Serial.write(*result); Serial.println(); return true;
  }
  Serial.println(F("  [timeout]")); return false;
}

// Helper: print a seconds value as days with 2 decimal places.
static void printDays(uint32_t sec) {
  if (sec == 0) { Serial.print(F("never")); return; }
uint32_t whole = sec / 86400;
  uint32_t frac  = (uint32_t)(((uint64_t)(sec % 86400) * 1000000ULL) / 86400ULL);  // six decimal digits
  Serial.print(whole); Serial.print('.');
  if (frac < 100000UL) Serial.print('0');
  if (frac < 10000UL)  Serial.print('0');
  if (frac < 1000UL)   Serial.print('0');
  if (frac < 100UL)    Serial.print('0');
  if (frac < 10UL)     Serial.print('0');
  Serial.print(frac);
  Serial.print(F(" days"));
}

// Helper: print a byte value as GB with 2 decimal places.
static void printGB(uint32_t bytes) {
  if (bytes == 0) { Serial.print(F("never")); return; }
  // bytes / 1e9 as fixed-point: whole = bytes/1000000000, frac = remainder*100/1e9
  uint32_t whole = bytes / 1000000000UL;
  uint32_t rem   = bytes % 1000000000UL;
  uint32_t frac  = (uint32_t)((uint64_t)rem * 1000000ULL / 1000000000ULL);  // six decimal digits
  Serial.print(whole); Serial.print('.');
  if (frac < 100000UL) Serial.print('0');
  if (frac < 10000UL)  Serial.print('0');
  if (frac < 1000UL)   Serial.print('0');
  if (frac < 100UL)    Serial.print('0');
  if (frac < 10UL)     Serial.print('0');
  Serial.print(frac);
  Serial.print(F(" GB"));
}

// ===========================================================================
//  CROSS-TRIGGER PARTNER LINK (3-wire: TRIG_OUT, TRIG_IN, shared GND)
// ===========================================================================

// Blink one LED: start HIGH, toggle every toggleMs, run for totalMs, end LOW.
static void blinkLed(byte pin, uint16_t toggleMs, uint16_t totalMs) {
  uint32_t start = millis(); uint32_t next = start + toggleMs; bool on = true;
  digitalWrite(pin, HIGH);
  while ((uint32_t)(millis() - start) < totalMs) {
    if ((int32_t)(millis() - next) >= 0) { on = !on; digitalWrite(pin, on ? HIGH : LOW); next += toggleMs; }
  }
  digitalWrite(pin, LOW);
}

// One-time pin setup for the link. TRIG_OUT push-pull idling LOW (active high),
// TRIG_IN with an internal pulldown so it reads LOW when no partner is connected.
static void configTriggerPins() {
  pinMode(PIN_TRIG_OUT, OUTPUT);
  digitalWrite(PIN_TRIG_OUT, LOW);          // active-high convention: LOW = idle
  pinMode(PIN_TRIG_IN, INPUT_PULLDOWN);     // defined LOW when nothing is driving it
}

// Arm the CHANGE interrupt that latches the partner line into partnerLineHigh.
// Only the Arduino wrappers are used (a hand-built am_hal pin config can assert
// on pads without a HW pull-down). attachInterrupt drops the pad pull, so the
// pull-down is re-applied after - which does not disarm the interrupt. The ISR
// reads the real level on every edge, so it self-corrects and never sticks; with
// a partner wired its push-pull output drives the line both ways, so detection
// never depends on the pull (it only matters standalone). Runs once at attach.
static void attachPartnerInterrupt() {
  pinMode(PIN_TRIG_IN, INPUT_PULLDOWN);
  attachInterrupt(PIN_TRIG_IN, partnerISR, CHANGE);
  pinMode(PIN_TRIG_IN, INPUT_PULLDOWN);                  // re-apply pull (interrupt stays armed)
  partnerLineHigh = (digitalRead(PIN_TRIG_IN) == HIGH);  // sync initial level
  if (partnerLineHigh) partnerSeen = true;
}

// Apply the AUTO/OFF setting:
//   AUTO - listen (interrupt armed) and drive TRIG_OUT from our own events.
//   OFF  - stop listening (detach) AND stop driving TRIG_OUT (fully standalone).
static void applyPartnerMode() {
  if (partnerAuto()) {
    attachPartnerInterrupt();
    if (selfActive) trigOutHigh(); else trigOutLow();   // output reflects own-event state
  } else {
    detachInterrupt(PIN_TRIG_IN);
    partnerLineHigh = false;
    trigOutLow();                                       // release the line
  }
}

// Startup handshake (in setup(), AUTO only): TRIG_OUT is driven HIGH from power-
// on and held through the whole startup so a co-booting partner always sees it,
// with the interrupt armed early to catch a partner HIGH at any point. A blue
// confirmation blink fires at the end of startup if a partner was seen, else it
// is armed to fire after the first live partner event.

// ===========================================================================
//  MENU SCREENS
// ===========================================================================
static void printMenu() {
  const char* fss[] = {"+/-2g","+/-4g","+/-8g","+/-16g"};
  const char* led[] = {"Normal LEDs","15s heartbeat","After-event flash","Off after boot"};
  uint8_t lm = settings.ledMode; if (lm > 3) lm = 2;

  // Live rate from the last RATE_RING_N sample timestamps (not the file average).
  uint32_t rate = 0;
  if (rateRingCount >= 2) {
    uint8_t newest = (uint8_t)((rateRingIdx + RATE_RING_N - 1u) % RATE_RING_N);
    uint8_t oldest = (rateRingCount < RATE_RING_N) ? 0 : rateRingIdx;
    uint32_t span = rateRing[newest] - rateRing[oldest];     // micros; wrap-safe over a few ms
    uint32_t cnt  = (uint32_t)rateRingCount - 1u;
    if (span > 0) rate = (uint32_t)(((uint64_t)cnt * 1000000ULL) / span);
  }

  Serial.println(F("\r\n====== OLA Accel Logger v1.0  CONFIG ======"));
  Serial.print(F("  Samples: ")); Serial.print(sampleCount);
  Serial.print(F("   ~")); Serial.print(rate); Serial.println(F(" Hz (last 20 samples)"));
  Serial.print(F("  CPU: ")); Serial.println(burstMode ? F("96 MHz (burst)") : F("48 MHz (normal)"));
  Serial.print(F("  Read path: ")); Serial.println(useRawRead ? F("RAW (accel-only)") : F("library getAGMT"));
  Serial.print(F("  Ready poll: ")); Serial.println(useGpioReady ? F("GPIO INT pin") : F("SPI dataReady()"));
  Serial.print(F("  File: ")); Serial.print(dataFileName);
  Serial.print(F("  (")); Serial.print(fileBytesWritten); Serial.println(F(" bytes)"));
  Serial.println(F("  Output: micros, ax, ay, az  (accel = milli-g x100)"));
  Serial.println(F(""));
  Serial.print(F("  1  Sampling rate         = ")); 
  settings.usBetweenReadings ? Serial.print(1000000.0/settings.usBetweenReadings) : Serial.print(F("inf (max)")); 
  Serial.print(F(" Hz  (~")); Serial.print(RATE_NORMAL_HZ);
  Serial.print(F(" Hz to ~")); Serial.print(RATE_BOOST_HZ);
  Serial.println(F(" Hz peak)"));
  Serial.print(F("  2  Record events only    = ")); Serial.print(settings.recordEventsOnly ? F("ON ") : F("OFF"));
  Serial.println(settings.recordEventsOnly ? F("  (save shock windows only)") : F("  (continuous)"));
  if (settings.recordEventsOnly) {
    Serial.print(F("       window=")); Serial.print(settings.eventWindowMs);
    Serial.print(F(" ms (")); Serial.print(settings.evPrePct); Serial.print(F("% pre / "));
    Serial.print(100 - settings.evPrePct); Serial.print(F("% post)  thresh="));
    Serial.print(settings.eventThreshMg, 1); Serial.println(F(" mg"));
    Serial.print(F("       Partner event trigger: "));
    if (settings.partnerMode == 0) Serial.println(F("OFF (standalone)"));
    else { Serial.print(F("AUTO")); Serial.println(partnerPaired ? F(" (paired)") : (partnerSeen ? F(" (partner detected)") : F(" (listening, no partner yet)"))); }
  }
  Serial.print(F("  3  Terminal output       = ")); Serial.println(settings.enableTerminalOutput ? F("ON") : F("OFF"));
  Serial.print(F("  4  Full-scale (FSS)      = ")); Serial.print(settings.imuAccFSS);
  Serial.print(F("  (")); Serial.print(fss[settings.imuAccFSS & 3]); Serial.println(F(")"));
  Serial.print(F("  5  Digital Filter (DLPF) = "));
  if (settings.imuAccDLPF) {
    Serial.println(F("ENABLED"));
    Serial.print(F("  6  DLPF Bandwidth        = ")); Serial.print(settings.imuAccDLPFBW);
    switch(settings.imuAccDLPFBW) {
      case 0: case 1: Serial.println(F("  (246/265 Hz)")); break;
      case 2:  Serial.println(F("  (111/136 Hz)")); break;
      case 3:  Serial.println(F("  (50/69 Hz)"));   break;
      case 4:  Serial.println(F("  (24/34 Hz)"));   break;
      case 5:  Serial.println(F("  (12/17 Hz)"));   break;
      case 6:  Serial.println(F("  (6/8 Hz)"));     break;
      case 7:  Serial.println(F("  (473/499 Hz)")); break;
      default: Serial.println(); break;
    }
  } else {
    Serial.println(F("DISABLED"));
  }
  Serial.println(F("  7  File rotation        ->"));
  Serial.print(F("       time = ")); printDays(settings.openNewLogFilesAfter); Serial.println();
  Serial.print(F("       size = ")); printGB(settings.maxLogFileBytes); Serial.println();
  Serial.print(F("  8  LED mode              = ")); Serial.print(lm);
  Serial.print(F("  (")); Serial.print(led[lm]); Serial.println(F(")"));
  Serial.print(F("  9  CPU burst (TurboSPOT) = ")); Serial.print(burstMode ? F("ON ") : F("OFF"));
  Serial.println(burstMode ? F("  (96 MHz, ~2x (rate and power))") : F("  (48 MHz, standard rate and power)"));
  Serial.println(F(""));
  Serial.println(F("  r  Reset all to defaults"));
  Serial.println(F("  x  Save and exit (resume logging)"));
  Serial.println(F("\r\nSelect 1-9, r, or x:"));
}

// -- Record-events-only sub-menu (option 2) -----------------------------------
static void menuEventMode() {
  while (1) {
    uint32_t preMs  = (uint32_t)settings.eventWindowMs * settings.evPrePct / 100u;
    uint32_t postMs = settings.eventWindowMs - preMs;
    Serial.println(F("\r\n-- Record Events Only --"));
    Serial.print(F("  Status: ")); Serial.println(settings.recordEventsOnly ? F("ON") : F("OFF"));
    Serial.println(F("  ON  = buffer accel history; when a shock happens, save a window"));
    Serial.print(F("        around it with ")); Serial.print(settings.evPrePct);
    Serial.print(F("% pre-event + ")); Serial.print(100 - settings.evPrePct);
    Serial.println(F("% post-event capture."));
    Serial.println(F("        Each logged event starts with a 'DAY: <decimal day>' marker."));
    Serial.println(F("        Repeated shocks extend the post-event cutoff."));
    Serial.println(F("        Quiet time is not written."));
    Serial.println(F("  OFF = log every sample (continuous)."));
    Serial.println(F(""));
    Serial.print(F("  current window = ")); Serial.print(settings.eventWindowMs);
    Serial.print(F(" ms  (pre=")); Serial.print(preMs);
    Serial.print(F(" ms / post=")); Serial.print(postMs); Serial.println(F(" ms)"));
    Serial.print(F("  current thresh = ")); Serial.print(settings.eventThreshMg, 1);
    Serial.println(F(" mg  (|a| deviation from rest that triggers the event window)"));
    Serial.print(F("  current split  = ")); Serial.print(settings.evPrePct);
    Serial.print(F("% pre / ")); Serial.print(100 - settings.evPrePct); Serial.println(F("% post"));
    Serial.print(F("  ring buffer holds ~")); Serial.print(estimateRingSeconds(), 1);
    Serial.print(F(" s  (~")); Serial.print(effectiveSampleRateHz());
    Serial.println(F(" Hz effective sample rate right now)"));
    Serial.println(F(" note: a long event flushes then continues, this caps the base window)"));
    Serial.println(F("  -- Partner event trigger 3-wire link:"));
    Serial.println(F("           Logger #1          Logger #2"));
    Serial.println(F("        - OUT (PIN 23)  ->   IN  (PIN 11)"));
    Serial.println(F("        - IN  (PIN 11)  <-   OUT (PIN 23)"));
    Serial.println(F("        -  shared GND   <->   shared GND"));
    Serial.println(F("  Two loggers cross-trigger over GPIO. Each drives its"));
    Serial.println(F("  trig-out HIGH during its OWN event and watches trig-in via interrupt;"));
    Serial.println(F("  when EITHER fires, BOTH record the same window."));
    Serial.println(F("  AUTO = always listen + drive trig-out; auto-connects a"));
    Serial.println(F("         partner the moment one appears (default, safe)."));
    Serial.println(F("  OFF  = no trig-in listening and no trig-out signal."));
    Serial.print(F("  Status: "));
    if (settings.partnerMode == 0) Serial.println(F("OFF (standalone)"));
    else { Serial.print(F("AUTO")); Serial.println(partnerPaired ? F(" - paired") : (partnerSeen ? F(" - partner detected") : F(" - no partner yet"))); }
    Serial.println(F(""));
    Serial.println(F("  0 = events OFF     1 = events ON"));
    Serial.println(F("  w = set window     g = set threshold     s = set pre/post split"));
    Serial.println(F("  p = partner trigger AUTO/OFF     x = back"));

    char c;
    if (!readChar(&c)) return;
    if (c == 'x') return;
    else if (c == '0') { settings.recordEventsOnly = false; Serial.println(F("  events-only OFF (continuous logging)")); }
    else if (c == '1') { settings.recordEventsOnly = true;  Serial.println(F("  events-only ON")); }
    else if (c == 'p') {
      settings.partnerMode = (settings.partnerMode == 0) ? 1 : 0;
      applyPartnerMode();
      Serial.print(F("  Partner event trigger: "));
      Serial.println(settings.partnerMode == 0 ? F("OFF (in + out disabled)") : F("AUTO (listening + driving)"));
    }
    else if (c == 'w') {
      Serial.println(F("\r\n-- Window duration --"));
      Serial.print(F("  Current: ")); Serial.print(settings.eventWindowMs); Serial.println(F(" ms"));
      Serial.print(F("  Total event capture span. Split: ")); Serial.print(settings.evPrePct);
      Serial.print(F("% pre-event, ")); Serial.print(100 - settings.evPrePct); Serial.println(F("% post-event."));
      Serial.print(F("  e.g. 4000 = 4.0s total -> ")); Serial.print(4000UL * settings.evPrePct / 100u);
      Serial.print(F("ms before + ")); Serial.print(4000UL - 4000UL * settings.evPrePct / 100u); Serial.println(F("ms after."));
      Serial.print(F("  ")); Serial.print(MIN_WINDOW_MS); Serial.print(F(" ms min, max = ring buffer span (~"));
      Serial.print(estimateRingSeconds(), 1); Serial.println(F(" s)"));
      Serial.println(F("\r\n  Type ms + Enter, or x to cancel:"));
      uint32_t v;
      if (readNumber(&v)) {
        if (v < MIN_WINDOW_MS) { v = MIN_WINDOW_MS; Serial.print(F("  (raised to ")); Serial.print(MIN_WINDOW_MS); Serial.println(F(" ms min)")); }
        uint32_t ringMs = (uint32_t)(estimateRingSeconds() * 1000.0f);
        if (ringMs < MIN_WINDOW_MS) ringMs = MIN_WINDOW_MS;
        if (v > ringMs) { v = ringMs; Serial.println(F("  (clamped to ring-buffer span)")); }
        settings.eventWindowMs = v;
        cacheDerivedSettings();
        Serial.print(F("  set: ")); Serial.print(v);
        Serial.print(F(" ms  (pre=")); Serial.print(v * settings.evPrePct / 100u);
        Serial.print(F(" ms / post=")); Serial.print(v - v * settings.evPrePct / 100u); Serial.println(F(" ms)"));
      }
    }
    else if (c == 'g') {
      Serial.println(F("\r\n-- Event threshold --"));
      Serial.print(F("  Current: ")); Serial.print(settings.eventThreshMg, 1); Serial.println(F(" mg"));
      Serial.println(F("  Trigger when |a| deviates from 1g (rest) by this much."));
      Serial.println(F("  1000 = 1.0 g -> fires on |a| > 2g."));
      Serial.print(F("  ")); Serial.print(MIN_THRESH_MG, 1); Serial.print(F(" mg min, "));
      Serial.print(MAX_THRESH_MG, 0); Serial.println(F(" mg max (16g full-scale)."));
      Serial.println(F("\r\n  Type mg (decimal OK) + Enter, or x to cancel:"));
      double v;
      if (readFloat(&v)) {
        if (v < (double)MIN_THRESH_MG) { v = (double)MIN_THRESH_MG; Serial.println(F("  (raised to minimum)")); }
        if (v > (double)MAX_THRESH_MG) { v = (double)MAX_THRESH_MG; Serial.println(F("  (clamped to 16g max)")); }
        settings.eventThreshMg = (float)v;
        cacheDerivedSettings();
        Serial.print(F("  set: ")); Serial.print(settings.eventThreshMg, 1); Serial.println(F(" mg"));
      }
    }
    else if (c == 's') {
      Serial.println(F("\r\n-- Pre/post event split --"));
      Serial.print(F("  Current: ")); Serial.print(settings.evPrePct);
      Serial.print(F("% pre / ")); Serial.print(100 - settings.evPrePct); Serial.println(F("% post"));
      Serial.println(F("  How much of the window to capture BEFORE vs AFTER the trigger."));
      Serial.println(F("  20 = 20% before, 80% after (default)."));
      Serial.println(F("  50 = even split.   2-98 range."));
      Serial.println(F("\r\n  Type pre-event % + Enter, or x to cancel:"));
      uint32_t v;
      if (readNumber(&v)) {
        if (v < 2)  { v = 2;  Serial.println(F("  (raised to 2% min)")); }
        if (v > 98) { v = 98; Serial.println(F("  (clamped to 98% max)")); }
        settings.evPrePct = (uint8_t)v;
        cacheDerivedSettings();
        Serial.print(F("  set: ")); Serial.print(settings.evPrePct);
        Serial.print(F("% pre / ")); Serial.print(100 - settings.evPrePct); Serial.println(F("% post"));
      }
    }
  }
}

// -- File rotation sub-menu (option 7) ----------------------------------------
static void menuFileRotation() {
  while (1) {
    Serial.println(F("\r\n-- File Rotation --"));
    Serial.println(F("  Rotation happens when EITHER condition is met."));
    Serial.println(F("  7 Day or 3.8 GB file size is the limit for either."));
    Serial.println(F("  Set 0 to disable a trigger."));
    Serial.println(F(""));
    Serial.print(F("  t  Time   = ")); printDays(settings.openNewLogFilesAfter); Serial.println();
    Serial.print(F("  s  Size   = ")); printGB(settings.maxLogFileBytes); Serial.println();
    Serial.println(F(""));
    Serial.println(F("  Type t, s, or x to return to main menu:"));

    char c;
    if (!readChar(&c)) return;
    if (c == 'x') return;

    else if (c == 't') {
      Serial.println(F("\r\n-- Time-based rotation --"));
      Serial.print(F("  Current: ")); printDays(settings.openNewLogFilesAfter); Serial.println();
      Serial.println(F("  0     = never time-rotate"));
      Serial.println(F("  >0    = new file after N days (decimal OK, 1 min minimum)"));
      Serial.println(F("  1     = 1 day    0.5 = 12 hours    7 = 1 week (max)"));
      Serial.println(F("\r\n  Type days + Enter, or x to cancel:"));
      double d;
      if (readFloat(&d)) {
        if (d < 0.0) d = 0.0;
        uint32_t sec = (uint32_t)(d * 86400.0 + 0.5);   // round to nearest second
        if (sec > 0 && sec < MIN_FILE_SECONDS) { sec = MIN_FILE_SECONDS; Serial.println(F("  (raised to 1 minute minimum)")); }
        if (sec > MAX_LOGFILE_SECONDS_CAP) { sec = MAX_LOGFILE_SECONDS_CAP; Serial.println(F("  (clamped to 7 days)")); }
        settings.openNewLogFilesAfter = sec;
        Serial.print(F("  set: ")); printDays(sec); Serial.println();
      }
    }

    else if (c == 's') {
      Serial.println(F("\r\n-- Size-based rotation --"));
      Serial.print(F("  Current: ")); printGB(settings.maxLogFileBytes); Serial.println();
      Serial.println(F("  0     = never size-rotate"));
      Serial.println(F("  >0    = new file at N GB (decimal OK, 25 KB minimum)"));
      Serial.println(F("  1     = 1 GB    0.2 = 200 MB    3.8 = 3.8 GB (max)"));
      Serial.println(F("\r\n  Type GB + Enter, or x to cancel:"));
      double d;
      if (readFloat(&d)) {
        if (d < 0.0) d = 0.0;
        uint32_t bytes = (uint32_t)(d * 1000000000.0 + 0.5);  // round to nearest byte
        if (bytes > 0 && bytes < MIN_FILE_BYTES) { bytes = MIN_FILE_BYTES; Serial.println(F("  (raised to 25 KB minimum)")); }
        if (bytes > MAX_LOGFILE_BYTES_CAP) { bytes = MAX_LOGFILE_BYTES_CAP; Serial.println(F("  (clamped to 3.8 GB)")); }
        settings.maxLogFileBytes = bytes;
        Serial.print(F("  set: ")); printGB(bytes); Serial.println();
      }
    }
  }
}

void menuMain() {
  sdFlushSectors(true);
  if (dataFile) { dataFile.sync(); stampWrite(&dataFile); }
  // Release the cross-trigger line while parked in the menu (don't hold a
  // partner recording). The interrupt stays armed so we still track the line.
  selfActive = false; recordingPrev = false; trigOutLow();
  afterFlashOn = false; hsFlashRunning = false; holdEndFlashRunning = false;  // drop any in-flight flash
  while (Serial.available()) Serial.read();
  delay(10); menuLedsOn(); printMenu();

  uint32_t idleStart = millis();
  while (1) {
    if ((millis()-idleStart)/1000 >= MENU_TIMEOUT_SEC) { Serial.println(F("\r\nMenu timeout.")); break; }
    if (!Serial.available()) continue;
    char c = Serial.read(); idleStart = millis(); menuBlink();

    if (c == 'x' || c == 'X') break;
    else if (c=='\r' || c=='\n') continue;

    else if (c == 'r' || c == 'R') {
      settings = Settings(); setAccelScale();
      clampSettings(); cacheDerivedSettings(); applyBurstMode(); applyPartnerMode();
      Serial.println(F("\r\n  All settings reset to firmware defaults."));
      printMenu();
    }

    else if (c == '1') {
      Serial.println(F("\r\n-- Sample Rate --"));
      Serial.print(F("  Current: "));
      settings.usBetweenReadings ? Serial.print(1000000.0 / settings.usBetweenReadings) : Serial.print(F("inf"));
      Serial.println(F(" Hz\n"));
      Serial.println(F("Input:  99999 = maximum rate    number = sample rate in Hz (decimal OK)"));
      Serial.print(F(" (~")); Serial.print(RATE_NORMAL_HZ);
      Serial.print(F(" Hz peak or ~")); Serial.print(RATE_BOOST_HZ);
      Serial.println(F(" Hz in CPU Burst Mode)"));
      if (settings.recordEventsOnly) {
        Serial.print(F("  Event mode: minimum ")); Serial.print(MIN_EVENT_RATE_HZ);
        Serial.println(F(" Hz (for 16-bit delta encoding)"));
      }
      Serial.println(F("\r\n  Type value + Enter, or x to cancel:"));
      double val; bool ok = false;
      if (readFloat(&val)) {
        if (val >= 99999.0) {
          settings.usBetweenReadings = 0;          ok = true;
        } else if (val > 0.0) {
          settings.usBetweenReadings = (uint32_t)(1000000.0 / val);
          // Event mode: enforce 20 Hz floor
          if (settings.recordEventsOnly) {
            uint32_t minUs = 1000000UL / MIN_EVENT_RATE_HZ;
            if (settings.usBetweenReadings > minUs) {
              settings.usBetweenReadings = minUs;
              Serial.print(F("  (clamped to ")); Serial.print(MIN_EVENT_RATE_HZ);
              Serial.println(F(" Hz event-mode minimum)"));
            }
          }
          ok = true;
        }
      }
      if (ok) {
        Serial.print(F("  set: "));
        settings.usBetweenReadings ? Serial.print(1000000.0 / settings.usBetweenReadings) : Serial.print(F("inf"));
        Serial.print(F(" Hz  ("));
        Serial.print(settings.usBetweenReadings);
        Serial.println(F(" us)"));
      }
      printMenu();
    }

    else if (c == '2') { menuEventMode(); printMenu(); }

    else if (c == '3') {
      settings.enableTerminalOutput ^= 1;
      Serial.print(F("\r\n  Terminal output ")); Serial.println(settings.enableTerminalOutput ? F("ON (slows logging)") : F("OFF"));
      printMenu();
    }

    else if (c == '4') {
      Serial.println(F("\r\n-- Full-Scale Range --"));
      Serial.println(F("  0: +/-2g   1: +/-4g   2: +/-8g   3: +/-16g"));
      Serial.println(F("\r\n  Type 0-3 + Enter, or x to cancel:"));
      uint32_t val;
      if (readNumber(&val) && val <= 3) {
        settings.imuAccFSS = (int)val;
        ICM_20948_fss_t fss; fss.a = settings.imuAccFSS; fss.g = 0;
        myICM.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);
        selectBank0(); setAccelScale(); cacheDerivedSettings();
        const char* s[] = {"+/-2g","+/-4g","+/-8g","+/-16g"};
        Serial.print(F("  set: ")); Serial.print((int)val);
        Serial.print(F(" (")); Serial.print(s[val]); Serial.println(F(")"));
      }
      printMenu();
    }

    else if (c == '5') {
      Serial.println(F("\r\n-- Digital Low-Pass Filter --"));
      Serial.println(F("  0: Disable    1: Enable"));
      uint32_t val;
      if (readNumber(&val) && val <= 1) {
        settings.imuAccDLPF = (bool)val;
        myICM.enableDLPF(ICM_20948_Internal_Acc, settings.imuAccDLPF);
        selectBank0();
        Serial.print(F("  DLPF ")); Serial.println(settings.imuAccDLPF ? F("ENABLED") : F("DISABLED"));
      }
      printMenu();
    }

    else if (c == '6') {
      if (!settings.imuAccDLPF) {
        Serial.println(F("\r\n  DLPF is disabled. Enable it first with option 5."));
      } else {
        Serial.println(F("\r\n-- DLPF Bandwidth --"));
        Serial.println(F("  0/1: 246 Hz  2: 111 Hz  3: 50 Hz  4: 24 Hz"));
        Serial.println(F("  5: 12 Hz  6: 6 Hz  7: 473 Hz"));
        Serial.println(F("\r\n  Type 0-7 + Enter, or x to cancel:"));
        uint32_t val;
        if (readNumber(&val) && val <= 7) {
          settings.imuAccDLPFBW = (int)val;
          ICM_20948_dlpcfg_t dlp; dlp.a = (int)val; dlp.g = 0;
          myICM.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlp);
          selectBank0();
          Serial.print(F("  DLPF BW set: ")); Serial.println((int)val);
        }
      }
      printMenu();
    }

    else if (c == '7') { menuFileRotation(); printMenu(); }

    else if (c == '8') {
      Serial.println(F("\r\n-- LED Mode --"));
      Serial.println(F("  0: Power LED on and flash on logging   1: Low power heartbeat (flash every 15 sec)"));
      Serial.println(F("  2: Flash after events only (default)   3: Ultra low (no lights ever)     "));
      uint32_t val;
      if (readNumber(&val) && val <= 3) {
        settings.ledMode = (uint8_t)val;
        const char* n[] = {"Normal LEDs","15s heartbeat","After-event flash","Off after boot"};
        Serial.print(F("  set: ")); Serial.print((int)val);
        Serial.print(F(" (")); Serial.print(n[val]); Serial.println(F(")"));
      }
      menuLedsOn(); printMenu();
    }

    else if (c == '9') {
      Serial.println(F("\r\n-- CPU Burst Mode (TurboSPOT) --"));
      Serial.println(F("  0: OFF (48 MHz, standard rate and power)      1: ON (96 MHz, ~2x (rate and power))"));
      uint32_t val;
      if (readNumber(&val) && val <= 1) { settings.burstEnabled = (bool)val; applyBurstMode(); }
      printMenu();
    }
  }

  recordSettings(); clampSettings(); cacheDerivedSettings();
  menuLedsOff(); if (settings.ledMode == 0) pwrLedOn(); drainSerial();
  lastRotateMillis = millis(); lastWriteMicros = micros(); lastLedFlashMillis = millis();
  sdBufLen = 0; sectorsSinceSync = 0;
  // Re-arm the event engine and the cross-trigger output for a clean restart.
  selfActive = false; recordingPrev = false; selfPostUntil = micros();
  evCapStart = evHead; evCapLen = 0; evPartnerInWindow = false;
  trigOutLow();
  if (partnerAuto()) partnerLineHigh = (digitalRead(PIN_TRIG_IN) == HIGH);  // resync line
  partnerHsPulse = false; partnerPulseCount = 0;            // drop any stale pairing pulse
  rateRingCount = 0;                                         // re-estimate rate from fresh samples
  hpfSettling = true; logStartMillis = millis();             // re-settle the HPF threshold
  afterFlashOn = false; hsFlashRunning = false; holdEndFlashRunning = false; statLedOff(); // clear any in-flight flash
  hpLpAx = (int32_t)rawAx << 16; hpLpAy = (int32_t)rawAy << 16; hpLpAz = (int32_t)rawAz << 16;
  Serial.println(F("\r\n  Settings saved. Returning to logging now.\r\n"));
  uint64_t hsDummy = 0; startupLedSequence(&hsDummy, false);   // visual only; never re-pairs from the menu
}

static bool startupLedSequence(uint64_t *completeRaw, bool announce) {
  // In event+AUTO mode, announce by TOGGLING TRIG_OUT (initiator role) while the
  // LED sequence runs, watching for a partner to toggle back. Stops on a LOW the
  // instant the partner is seen toggling; returns whether we paired.
  bool toggling = announce && settings.recordEventsOnly && partnerAuto();
  bool paired = false;
  if (toggling) { partnerPulseCount = 0; partnerHsPulse = false; }  // fresh detection baseline
  uint32_t tNx = micros(); bool tout = false;
  uint32_t start = millis(); bool s = 0, p = 0; uint32_t ts = start;
  while (millis() - start < 2000) {
    if (millis() >= ts) { s=!s; s?statLedOn():statLedOff(); ts+=250; p=!p; p?pwrLedOn():pwrLedOff(); }
    if (toggling && hsInitiatorStep(&tNx, &tout, completeRaw)) { toggling = false; paired = true; }
  }
  start = millis(); ts = start; uint32_t tp = start + 40;
  while (millis() - start < 2000) {
    if (millis() >= ts) { s=!s; s?statLedOn():statLedOff(); ts+=40; }
    if (millis() >= tp) { p=!p; p?pwrLedOn():pwrLedOff();   tp+=40; }
    if (toggling && hsInitiatorStep(&tNx, &tout, completeRaw)) { toggling = false; paired = true; }
  }
  if (toggling) trigOutLow();                                // announce window ended, no partner
  statLedOff(); pwrLedOff();
  if (settings.ledMode == 0) pwrLedOn();
  return paired;
}

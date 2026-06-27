/*
  OLA_Accel_Logger  v1.0  — accelerometer shock / event logger for OpenLog Artemis
  SparkFun RedBoard Artemis ATP  +  ICM-20948 (accelerometer only).

  A focused rewrite of the stock OpenLog Artemis logger for high-rate impact and
  shock capture. The .ino holds only setup() and loop(); all configuration,
  state, the capture engine, the menu, and the partner link live in the .h.

  WHAT DIFFERS FROM STOCK OLA
    - Accelerometer-only: all gyro / magnetometer / temperature config and logging
      removed (the IMU's other sensors are left at power-on defaults, unused).
    - Raw accel-only SPI read (6 bytes), boot-verified against the library and
      falling back to it on mismatch. GPIO data-ready polling, likewise verified.
      Integer-only formatting and cached scaling - no float in the hot path.
    - Event-capture mode: a rolling RAM ring keeps pre-trigger history and only a
      window around each shock is written (most deployments sit idle for hours).
    - 3-wire partner cross-trigger: two loggers capture the same window when EITHER
      sees a shock, pairing over the same wires with a toggle handshake.
    - Per-event decimal-day timestamping, configurable CPU burst (96 MHz), and a
      startup self-test of the achievable sample rate.

  ============================================================================
   OUTPUT  (CSV, header "micros,aX,aY,aZ")
   Acceleration is milli-g x100 as an integer: /100 = mg, /100000 = g.
   e.g. 98100 = 981.00 mg = 0.981 g.  Full-scale range +/-1,600,000 (= +/-16 g).

   Continuous mode: one line per sample. First column = micros() at capture
   (uint32, wraps ~71 min).

   Event mode: only a window around each shock is written, each window prefixed by a
   "DAY:" marker = the absolute decimal day (12 dp) of the window's first sample,
   measured from logging start and re-zeroed at a partner handshake. Inside the
   window the first column is a 16-bit microsecond offset from that marker: it
   starts at 0 and rolls over at 65536. Reconstruct absolute time downstream by
   accumulating the offset (add 65536 on each decrease) and adding it to the DAY
   value.
       micros,aX,aY,aZ
       DAY: 0.123456789012
       0,10279,1298,-10982
       204,10281,1301,-10979
       ...                      (one event window)
       DAY: 1.622131298745
       ...                      (next event window)

  ============================================================================
   MENU  (send any character over serial to open; auto-saves and resumes on exit)
     1  Sampling rate        min us between samples (0 = max; event mode floor 20 Hz)
     2  Record events only   OFF = continuous; ON = shock windows only. Submenu sets
                             window length, threshold, pre/post split, partner mode
     3  Terminal output      echo CSV to serial (slows logging)
     4  Full-scale (FSS)     +/-2 / 4 / 8 / 16 g
     5  Digital filter DLPF  on/off (off = full bandwidth, max ODR)
     6  DLPF bandwidth       3 dB corner when DLPF is on
     7  File rotation        new file after N days and/or N GB
     8  LED mode             normal / heartbeat / after-event (default) / off
     9  CPU burst TurboSPOT  48 vs 96 MHz
     r  reset to defaults      x  save and resume

  ============================================================================
   EVENT MODE (option 2)
   A rolling ring buffers recent accel samples in RAM; nothing is written until
   |a| deviates from rest (1 g) by more than the threshold. The saved window spans
   evPrePct% before + (100-evPrePct)% after the trigger (default 20/80). Repeated
   shocks within the trailing guard extend the window, so a full post-roll of quiet
   always follows the last shock.
     - Trigger: a single-pole high-pass removes gravity, then |a_hp|^2 is compared
       to threshold^2 (no sqrt, no float). The deviation test avoids the constant
       firing a bare ">1 g" test would cause at rest.
     - Settling: for ~2 s after logging starts the OWN threshold is muted while the
       high-pass settles, so gravity removal can't fire a false event. Partner
       triggering is never muted.
     - Window length is bounded by the RING, not a fixed time. A long event that
       fills the ring is flushed to SD mid-event (a brief sampling gap) and capture
       continues in a fresh chunk, each chunk with its own DAY marker. After each
       flush the threshold is re-checked, so the window ends promptly once the shock
       subsides.
     - Files never rotate mid-event; rotation is deferred until the window closes,
       and a time-rotate that would make an empty file (header only) is skipped.

  ============================================================================
   PARTNER CROSS-TRIGGER  (3-wire: TRIG_OUT pin 32 -> partner TRIG_IN pin 11,
   TRIG_IN <- partner TRIG_OUT, shared GND. No external parts.)
     AUTO (default): always listen and drive; a partner auto-connects at startup or
                     any time during logging.
     OFF           : input and output both disabled; fully standalone.

   Cross-trigger (active-high): TRIG_OUT idles LOW and is driven HIGH only while
   THIS unit's own threshold window is live; TRIG_IN HIGH means the partner's window
   is live. A CHANGE interrupt latches the partner level into a RAM flag, so the hot
   loop never polls GPIO and an edge is caught even mid-SD-flush. The saved window
   is the UNION of both: it opens on the first HIGH and closes only when both are
   LOW. TRIG_OUT follows our own event only, so two units can't latch each other on.

   Pairing handshake (same two wires): a STEADY high is an event, a rapid ~250 us
   TOGGLE is a pairing attempt - a HIGH that falls within 1 ms is a pairing pulse,
   not an event, and two pulses confirm a real partner. A unit announces by toggling
   TRIG_OUT and watching for the partner to toggle back; it stops the instant it
   sees the partner, who keeps toggling until this unit goes quiet (~1 ms). On a
   runtime connect the running unit flushes any open window before pairing.

   On a successful handshake BOTH units zero the per-event DAY clock at the
   handshake instant (synced to ~1 ms), give a quick blue STAT blink to confirm the
   link, then suppress event starts for 5 s and fire one low-power STAT flash -
   timed to land within ~1 ms on the pair - to mark logging going live. A unit that
   boots AUTO with no partner present defers that blink to its first partner event.

  ============================================================================
   CPU BURST / POWER (option 9)
   96 MHz roughly doubles the achievable sample rate. The CPU delta is tiny
   (~+0.3 mA) but ~2x the rate means ~2x SD writes, and the card dominates draw,
   so continuous-mode current rises toward ~2x. In event mode the card is idle
   between events, so burst's cost there scales with event frequency, not clock.
   Default ON; turn OFF for the longest battery life.

   STARTUP RATE CHECK: just before logging, the firmware samples for 1 s and
   restarts itself if the measured rate falls below 90% of the expected rate
   (a guard against an occasional slow-start of the read path).

  ============================================================================
   DESIGN NOTES (intentional, hardware-specific)
   - IMU chip-select uses digitalWrite (raw fast-GPIO is unreliable on this pad).
   - Accel read is 6 single-byte SPI transfers (the sequence the hardware accepts
     here; SPI is fixed at 48 MHz, so burst doesn't speed the transfer itself).
   - SD preallocation is OFF (avoids stale sectors after an unclean power loss).

  Board:  SparkFun Apollo3 -> RedBoard Artemis ATP
  Libs:   SparkFun ICM-20948 IMU, SdFat v2.x exFAT, Apollo3 core
*/

#include "OLA_Accel_Logger.h"

// ===========================================================================
//  SETUP
// ===========================================================================
void setup() {
  pinMode(PIN_POWER_LOSS, INPUT); delay(1);
  if (digitalRead(PIN_POWER_LOSS) == LOW) emergencyFlush();
  attachInterrupt(PIN_POWER_LOSS, powerLossISR, FALLING);
  powerLossSeen = false;
  pinMode(PIN_STAT_LED, OUTPUT); statLedOn();
  pinMode(PIN_PWR_LED, OUTPUT);
  pin_config(PinName(PIN_PWR_LED), g_AM_HAL_GPIO_OUTPUT); pwrLedOn();
  pinMode(PIN_QWIIC_POWER, OUTPUT);
  pin_config(PinName(PIN_QWIIC_POWER), g_AM_HAL_GPIO_OUTPUT); qwiicPowerOff();
  SPI.begin();
  Serial.begin(115200);
  Serial.printf("\r\nOLA Accel Logger v%d.%d\r\n", FW_MAJOR, FW_MINOR);

  // --- Cross-trigger: listen from power-on -----------------------------------
  // Arm the CHANGE interrupt immediately so a partner's pairing TOGGLE is caught
  // at any point during startup (the ISR flags it via partnerHsPulse). TRIG_OUT
  // idles LOW now; we announce by toggling it during the LED sequence, once SD and
  // the IMU are up. Mode isn't known until loadSettings; the interrupt is detached
  // below if the loaded mode is OFF.
  partnerSeen = false; partnerLineHigh = false;
  partnerPulseCount = 0; partnerHsPulse = false; partnerPaired = false;
  configTriggerPins();                               // TRIG_OUT idles LOW (no steady-HIGH announce)
  attachPartnerInterrupt();                          // listen from power-on; the ISR detects pulses

  EEPROM.init();
  beginSD();
  enableCIPOpullUp();
  loadSettings();
  applyBurstMode();

  // Reconcile the announce/listen with the loaded mode. OFF -> stop both now
  // (drop the brief startup HIGH and detach the interrupt); AUTO -> leave
  // TRIG_OUT HIGH and the interrupt armed for the rest of startup.
  if (!partnerAuto()) {
    detachInterrupt(PIN_TRIG_IN);
    partnerLineHigh = false; partnerSeen = false;
    trigOutLow();
  }

  if (settings.terminalBaudRate != 115200) { Serial.flush(); Serial.begin(settings.terminalBaudRate); }
  Serial.println(F("Allowing time to settle power rails..."));
  delay(1000);
  beginIMU();
  lastRotateMillis = millis(); lastWriteMicros = micros(); lastLedFlashMillis = millis();
  bootMillis = millis(); bootMicros = micros(); prevMicros = (uint32_t)bootMicros;
  usHigh = 0; sampleCount = 0;
  rateRingIdx = 0; rateRingCount = 0;                 // fresh live-rate estimate
  // Event engine + flash state to a clean armed/idle baseline.
  selfActive = false; recordingPrev = false; selfPostUntil = micros();
  evPartnerInWindow = false; afterFlashOn = false; hsFlashRunning = false;
  evHead = 0; evCapStart = 0; evCapLen = 0;
  hpLpAx = (int32_t)rawAx << 16; hpLpAy = (int32_t)rawAy << 16; hpLpAz = (int32_t)rawAz << 16;
  statLedOff(); if (settings.ledMode >= 1) pwrLedOff();
  Serial.println(F("Logging started. Send any char to open menu."));
  Serial.printf("  usBetweenReadings=%lu us  ledMode=%d\r\n",
                (unsigned long)settings.usBetweenReadings, (int)settings.ledMode);
  Serial.printf("  events-only=%d  window=%lu ms (%d%%pre/%d%%post)  thresh=",
                (int)settings.recordEventsOnly,
                (unsigned long)settings.eventWindowMs,
                (int)settings.evPrePct, (int)(100 - settings.evPrePct));
  Serial.print(settings.eventThreshMg, 1); Serial.println(F(" mg"));
  Serial.print(F("  rotate: time=")); printDays(settings.openNewLogFilesAfter);
  Serial.print(F("  size=")); printGB(settings.maxLogFileBytes); Serial.println();
  Serial.print(F("  CPU: ")); Serial.println(burstMode ? F("96 MHz (burst)") : F("48 MHz (normal)"));
  Serial.print(F("  Partner event trigger: ")); Serial.println(partnerAuto() ? F("AUTO (default)") : F("OFF"));
  Serial.println(F(""));
  // Run the LED startup sequence; in event+AUTO mode this also TOGGLES TRIG_OUT to
  // announce and listens for a partner to toggle back (initiator). Returns whether
  // we paired, with the 64-bit handshake-completion micros in hsCompleteRaw.
  uint64_t hsCompleteRaw = 0;
  bool paired = startupLedSequence(&hsCompleteRaw, true);

  // Open (or reuse) the log file - after the LED sequence, before the confirm
  // blink. Reusing a header-only file means a reboot that logged nothing, or a
  // restart from the rate check below, won't waste a file. File creation happens
  // ONLY here and on rotation.
  openNextLogFile(true);

  // --- Cross-trigger handshake resolution ------------------------------------
  // If a partner paired during startup, zero the DAY clock at the handshake instant
  // (initiator side, no +1 ms), light the partner-mode confirm blink, and hold off
  // events for PARTNER_HOLD_MS. Otherwise (AUTO, no partner yet) arm a one-shot blue
  // blink to fire after the first live partner event - AUTO keeps listening later.
  hsFlashArmed = false;
  if (paired) {
    partnerResetDayClock(hsCompleteRaw, false);
    partnerPaired = true;
    armPartnerHold(hsCompleteRaw);                 // hold ends 5 s after the (startup) handshake instant
    blinkLed(PIN_STAT_LED, HS_BLINK_TOGGLE_MS, HS_BLINK_TOTAL_MS);
    partnerHsPulse = false; partnerPulseCount = 0; partnerLineHigh = false;  // drop the partner's residual handshake toggles so loop() cannot re-pair
  } else if (partnerAuto()) {
    hsFlashArmed = true;
  }

  // Startup rate check: sample for 1 s and verify the IMU/SPI path achieves
  // at least 90% of the expected rate. If not, flash LED and restart the system.
  // This runs BEFORE the HPF settling timer starts so the full 2 s of settling
  // happens after the check passes.
  startupRateCheck();

  // HPF threshold settling begins now (the OWN threshold is muted for the first
  // HPF_SETTLE_MS of logging so gravity removal can't fire a spurious event).
  hpfSettling = true; logStartMillis = millis();
}

// ===========================================================================
//  LOOP
// ===========================================================================
void loop() {
  if (powerLossSeen) emergencyFlush();
  // Partner pairing pulse seen (ISR flag)? Pair now (event+AUTO only). One volatile
  // read in the common case; the handler is semi-blocking but only runs on a pulse.
  if (partnerHsPulse && settings.recordEventsOnly && partnerAuto()) {
    if (inPartnerHold()) { partnerHsPulse = false; partnerPulseCount = 0; }  // partner's post-handshake echo toggles: ignore during the hold (no re-pair)
    else { handlePartnerConnect(); return; }
  }
  if (afterFlashOn || hsFlashRunning || holdEndFlashRunning) serviceLeds();   // skip millis() call when no flash is live
  if (Serial.available()) { menuMain(); return; }
  if (settings.usBetweenReadings > 0) {
    if ((uint32_t)(micros() - lastWriteMicros) < settings.usBetweenReadings) return;
  }
  uint32_t spinGuard = 0;
  while (!imuDataReady()) {
    if (powerLossSeen) emergencyFlush();
    if (++spinGuard >= SPIN_GUARD_MAX) return;
  }
  const uint32_t captureMicros = micros();
  if (captureMicros < prevMicros) usHigh++;
  prevMicros = captureMicros; lastWriteMicros = captureMicros;
  // Live-rate ring: store this sample's time (cheap; read back in the menu).
  rateRing[rateRingIdx] = captureMicros;
  if (++rateRingIdx >= RATE_RING_N) rateRingIdx = 0;
  if (rateRingCount < RATE_RING_N) rateRingCount++;
  const bool eventMode = settings.recordEventsOnly;
  readAccel(!eventMode); sampleCount++;                 // scale (mg x100) only when continuous
  if (eventMode) { eventModeStep(captureMicros); return; }

  const bool lm0 = (settings.ledMode == 0);            // cached: read twice below
  uint8_t *dst = sdBuf + sdBufLen;                      // build the CSV line in place (no memcpy)
  uint8_t  n   = buildLineInto(dst, captureMicros);
  if (settings.enableTerminalOutput) Serial.write(dst, n);
  if (lm0) statLedOn();
  sdBufLen += n; fileBytesWritten += n;
  if (sdBufLen >= SD_WRITE_THRESHOLD) sdFlushSectors(false);
  if (lm0) statLedOff();
  const uint32_t now = millis();
  bool rotateByTime = (rotateAfterMillis > 0) && (now - lastRotateMillis >= rotateAfterMillis);
  bool rotateBySize = (settings.maxLogFileBytes > 0) && (fileBytesWritten >= settings.maxLogFileBytes);
  if (rotateByTime || rotateBySize) rotateLogFile();
  if (settings.ledMode == 1) {
    static bool ledOn = false; static uint32_t ledStart = 0;
    if (!ledOn && (now - lastLedFlashMillis >= 15000UL)) {
      lastLedFlashMillis = now; ledStart = now; ledOn = true; statLedOn();
    }
    if (ledOn && (now - ledStart >= 2)) { statLedOff(); ledOn = false; }
  }
}

# OpenLog Artemis — Speed & Power-Optimized 3-Axis Accelerometer Remake

A focused rewrite of the stock [SparkFun OpenLog Artemis](https://www.sparkfun.com/products/16832)
firmware for **high-rate impact and shock capture**. It turns the OLA into a
consistent multi-kHz accelerometer logger with an **event-window capture mode**, a
**3-wire two-unit cross-trigger**, and per-event decimal-day timestamping — while
writing far less to the SD card.

Target hardware: **SparkFun RedBoard Artemis ATP (Apollo3)** + **ICM-20948** (accelerometer only).

![Stock firmware vs this remake](docs/images/04-old-vs-new.png)

*Stock OLA logs continuously at ~515 Hz with large gaps every 10–20 samples (slow SD
writes stall sampling). This remake samples consistently at ~5.4 kHz and only writes a
window around each event — ~54 samples per 10 ms instead of ~5.*

---

## Why

The off-the-shelf OLA logger writes every sample to the SD card inline, so the card's
write latency periodically stalls sampling — you get ~515 Hz with ragged gaps. For
impact and vibration work that loses the fast transient you care about.

This remake fixes both problems:

- **Consistent high sample rate** — the read path is reworked for ~3.2 kHz (48 MHz) or
  ~5.4 kHz (96 MHz "TurboSPOT" burst, default).
- **Event-window logging** — samples stream into a RAM ring; the SD card is only written
  *after* an event closes, by flushing a buffer rather than logging sample-by-sample. SD
  latency never interrupts capture, and you write 10–1000× less data.

## Highlights

- **Accelerometer-only** rewrite — gyro/mag/temperature config and logging removed.
- **Raw 6-byte SPI accel read**, boot-verified against the SparkFun library (falls back
  to it on mismatch). Integer-only formatting, cached scaling — **no float in the hot path**.
- **Event-capture mode** — a rolling pre-trigger buffer + a post-trigger window, extended
  when new shocks land inside the trailing guard.
- **3-wire partner cross-trigger** — two loggers capture the same window when *either*
  sees a shock, and pair over those same wires with a toggle handshake.
- **Per-event decimal-day timestamps**, configurable CPU burst, file rotation by time/size,
  and a **startup self-test** of the achievable sample rate.
- **Serial menu** for all settings; auto-saves to EEPROM and resumes.

---

## Performance

![Inter-sample-rate distribution](docs/images/03-sample-rate-distribution.png)

*Measured inter-sample rate. Peak throughput rises from **515 Hz** (stock) to **~3.2 kHz**
(48 MHz) and **~5.4 kHz** (96 MHz burst). The slow-sample tail on the left only exists in
continuous mode — event mode buffers and flushes after each window, so it never stalls
mid-event.*

| Firmware / mode            | Peak sample rate (measured) | SD behavior                          |
| -------------------------- | --------------------------- | ------------------------------------ |
| Stock OLA (continuous)     | ~515 Hz, gaps every 10–20   | writes every sample (stalls sampling)|
| This remake @ 48 MHz       | ~3.2 kHz (3165 Hz)          | buffered; flush after each event     |
| This remake @ 96 MHz burst | ~5.4 kHz (5525 Hz)          | buffered; flush after each event     |

---

## Event logging

![Anatomy of one logged event](docs/images/01-event-window.png)

*One logged event. A pre-trigger buffer that is already in RAM (default 20% of the window)
precedes the shock; an after-trigger tail (default 80%) follows. New shocks inside the
trailing 80% extend the window, so a full quiet post-roll always follows the last shock.
Saturation (full-scale) samples are flagged for downstream analysis. The screenshot is
configured to a 4 s window (800 ms pre + 3.2 s post).*

How a trigger is decided: a single-pole high-pass removes gravity, then |a_hp|² is compared
to threshold² (no sqrt, no float). For ~2 s after logging starts the *own* threshold is muted
while the high-pass settles; partner triggering is never muted. The window length is bounded
by the RAM ring, not a fixed time — a long event that fills the ring is flushed mid-event (a
brief sampling gap) and capture continues in a fresh chunk with its own `DAY:` marker.

![One day of event-only logging](docs/images/02-one-day-events.png)

*A full day from two units in event-only mode. Only windows around shocks are written, so
the day collapses to a handful of short records — the 10–1000× data reduction. Per-axis peak
accelerations from both loggers are annotated.*

---

## Hardware & wiring

| Signal     | Artemis ATP pin | Notes                                                        |
| ---------- | --------------- | ------------------------------------------------------------ |
| `TRIG_OUT` | **32**          | push-pull, idles LOW; driven HIGH while *this* event is live |
| `TRIG_IN`  | **11**          | interrupt + pulldown; HIGH = the partner's event is live     |
| `GND`      | GND             | shared between the two loggers                               |
| STAT LED   | 19 (blue)       | handshake / event flashes                                    |
| PWR LED    | 29 (red)        | status / rate-check-fail flash                               |

Two-unit cross-trigger is just three wires, no external parts:

```
   Logger A                          Logger B
   TRIG_OUT (32) ───────────────▶  TRIG_IN  (11)
   TRIG_IN  (11) ◀───────────────  TRIG_OUT (32)
   GND          ─────────────────  GND
```

Single-unit use needs no wiring — set partner mode to OFF (or leave AUTO; it simply never
finds a partner).

---

## Output format

CSV, header `micros,aX,aY,aZ`. Acceleration is **milli-g × 100** as an integer
(`/100` = mg, `/100000` = g); full-scale ±1,600,000 = ±16 g.

**Continuous mode** — one line per sample; column 1 is `micros()` at capture (uint32, wraps
~71 min).

**Event mode** — only a window around each shock is written. Each window is prefixed by a
`DAY:` marker = the absolute decimal day (12 dp) of the window's first sample, measured from
logging start and re-zeroed at a partner handshake. Inside a window, column 1 is a 16-bit
microsecond offset from that marker (starts at 0, rolls over at 65536). Reconstruct absolute
time downstream by accumulating the offset (add 65536 on each decrease) and adding it to the
`DAY` value.

```
micros,aX,aY,aZ
DAY: 0.123456789012
0,10279,1298,-10982
204,10281,1301,-10979
...                      (one event window)
DAY: 1.622131298745
...                      (next event window)
```

---

## Configuration (serial menu)

Send any character over serial (115200) to open the menu; it auto-saves to EEPROM and resumes
logging on exit.

| Key | Setting              | Notes                                                              |
| --- | -------------------- | ------------------------------------------------------------------ |
| 1   | Sampling rate        | min µs between samples (0 = max; event-mode floor 20 Hz)           |
| 2   | Record events only   | OFF = continuous; ON = shock windows only (+ window/threshold/split/partner submenu) |
| 3   | Terminal output      | echo CSV to serial (slows logging)                                 |
| 4   | Full-scale (FSS)     | ±2 / 4 / 8 / 16 g                                                   |
| 5   | Digital filter DLPF  | on/off (off = full bandwidth, max ODR)                             |
| 6   | DLPF bandwidth       | 3 dB corner when DLPF is on                                        |
| 7   | File rotation        | new file after N days and/or N GB                                  |
| 8   | LED mode             | normal / 15 s heartbeat / after-event (default) / off              |
| 9   | CPU burst (TurboSPOT)| 48 vs 96 MHz                                                        |
| r   | reset to defaults    |                                                                    |
| x   | save and resume      |                                                                    |

**Defaults:** event mode ON, ±16 g, 1 g trigger threshold, 3 s window (20% pre / 80% post),
max sample rate, 96 MHz burst, partner AUTO, after-event LED.

---

## Partner cross-trigger & handshake

During normal operation the link is **active-high**: `TRIG_OUT` is driven HIGH only while
*this* unit's threshold window is live, and the saved window is the **union** of both units —
it opens on the first HIGH and closes only when both are LOW. `TRIG_OUT` follows our own event
only, so two units can't latch each other on.

**Pairing** rides the same two wires. A *steady* HIGH is an event; a rapid ~250 µs *toggle* is
a pairing attempt (a HIGH that falls within 1 ms is a pairing pulse, not an event, and two
pulses confirm a real partner). A unit announces by toggling `TRIG_OUT` and watching for the
partner to toggle back; it stops the instant it sees the partner, who keeps toggling until this
unit goes quiet (~1 ms). A partner can pair at startup **or** any time during logging (on a
runtime connect the running unit flushes any open window first).

On a successful handshake **both** units zero the per-event DAY clock at the handshake instant
(synced to ~1 ms), give a quick blue STAT blink to confirm the link, then suppress event starts
for 5 s and fire one low-power STAT flash — timed to land within ~1 ms on the pair — to mark
logging going live.

---

## Build & flash

1. **Boards core** — in Arduino IDE → *Preferences → Additional Boards Manager URLs*, add:
   ```
   https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/main/package_sparkfun_apollo3_index.json
   ```
   then *Tools → Board → Boards Manager* → install **SparkFun Apollo3**. Select board
   **RedBoard Artemis ATP**.
2. **Libraries** (*Tools → Manage Libraries*):
   - **SparkFun 9DoF IMU Breakout - ICM 20948**
   - **SdFat** by Bill Greiman (v2.x — exFAT)
3. **Open the sketch** — open `OLA_Accel_Logger/OLA_Accel_Logger.ino` (the `.ino` and `.h`
   must stay together in the `OLA_Accel_Logger/` folder).
4. **Upload**, then open Serial Monitor at **115200** and send any character for the menu.

> Tested as firmware **v1.0**. The two-unit handshake/sync timing constants are tuned for this
> hardware; if you change boards or wiring, re-check them on a scope.

---

## Repository layout

```
.
├── OLA_Accel_Logger/
│   ├── OLA_Accel_Logger.ino     # setup() + loop() only
│   └── OLA_Accel_Logger.h       # config, state, capture engine, menu, partner link
├── docs/
│   └── images/                  # figures used in this README
├── LICENSE
└── README.md
```

---

## Companion analysis

![MATLAB alignment viewer](docs/images/05-alignment-viewer.png)

*Captured windows are reviewed with a companion MATLAB viewer that overlays the two loggers
(here "Crate" vs "SPIDERS"), steps through thresholded event windows, lets you slide one trace
to align it in time, and flags saturated samples excluded from PSDs. The firmware's CSV output
feeds this tool directly. (Analysis tool — not part of this firmware repository.)*

---

## Attribution & license

This project is a rewrite derived from SparkFun's open-source OpenLog Artemis firmware; please
keep upstream attribution if you redistribute. Released under the **MIT License** (see
[`LICENSE`](LICENSE)). This note is not legal advice — confirm compatibility with the upstream
license for your use.

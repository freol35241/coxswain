# hardware.md

The supported device list. This document is the fence against the device
zoo: a device appears here when it has been exercised on the bench, and
"quirks in configuration, not code" applies only to devices on this
list. Until the bench exists, the list below is split into what the code
supports by construction, what first water requires, and an unverified
procurement shortlist.

## Interfaces the code supports today

- NMEA 0183 over UART, listen-only, strict parsing (GGA, RMC, HDT, VTG;
  NMEA 2.3/4.1 layouts with FAA mode). The GNSS driver emits position
  (GGA, gated on fix quality, std from HDOP x UERE) and heading (HDT).
- CRSF over UART from an RC receiver (RC channels, link statistics).
  Kill switch, takeover switch, surge/yaw sticks per D-025.
- Actuator command out: one `$CXACT,<surge_n>,<sway_n>,<yaw_nm>*HH` line
  per 100 ms control tick over UART. The far end must fail safe on
  silence (recommended: zero thrust after 500 ms without a valid line).
- Hosted profile serial: standard POSIX bauds via termios; on Linux, any
  other exact rate (CRSF's 420000, notably) via termios2/BOTHER.

## What first water requires (from the 2026-07-10 replay experiment)

1. One 0183 GNSS compass emitting GGA and HDT on a single serial line,
   heading at 5 Hz or better. 1 Hz heading without a gyro is
   demonstrably unsafe (diary 2026-07-10); a dedicated IMU is off the
   critical path as long as this requirement holds.
2. An ExpressLRS (CRSF) receiver wired to the conn node, plus any ELRS
   transmitter. SBUS is the fallback if hardware dictates (parser not
   yet written; write it only if forced).
3. A far-end actuator controller: any small MCU that parses `$CXACT`
   and drives the vessel's ESC/servo hardware, failing safe on silence.
   This is the one piece of custom far-end firmware in the bring-up
   path; the Phase 9 actuator node replaces it.
4. Power monitoring reaching the failsafe matrix. No input path exists
   in the hosted real-serial mode yet (see gaps); decide the mechanism
   on the bench (candidates: an INA2xx-class monitor read by the
   actuator MCU and reported back, or a separate sensor line).
5. A Linux computer as the conn node for the hosted profile (Raspberry
   Pi 4/5 class or any industrial equivalent) with USB-serial adapters,
   or onboard UARTs. The NUCLEO-H753ZI enters at Phase 8, not first
   water.

## Procurement shortlist (candidates, unverified)

Not endorsements and not the fence: verify current model, sentence set,
heading rate, and output baud against the requirements above before
ordering anything.

| Need | Candidates to evaluate |
|---|---|
| 0183 GNSS compass, HDT at 5-10 Hz | Airmar GH2183, Hemisphere V200s-class, Furuno SC-family with 0183 output, Simrad HS-series |
| ELRS receiver + transmitter | RadioMaster ER-series or RP-series RX; Pocket/Boxer TX |
| Actuator MCU | Anything with a UART and PWM the shop already knows; RP2040 or an STM32 Nucleo both fine |
| Power monitor | INA226/INA228 breakout on the actuator MCU |
| Conn node | Raspberry Pi 5 + powered USB hub + FTDI-class serial adapters |

## Known gaps before real hardware

- Closed: CRSF's 420000 baud is not a POSIX `Bxxxx` rate; the hosted
  termios path now falls back to Linux's termios2/BOTHER ioctl pair for
  any rate outside that table (coxswain-hosted/src/serial.rs).
- No power monitoring input path in real-serial mode; the failsafe
  matrix currently sees a healthy default. Must be closed before any
  armed on-water operation.
- RC and actuator serial links are CLI options, not manifest buses;
  promoting them into the schema is an open item recorded in the code.
- Baud for the GNSS bus comes from the manifest bus entry; confirm the
  chosen compass actually speaks its rated sentences at that baud with
  heading at 5 Hz or better.

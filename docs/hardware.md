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
- NMEA 0183 over UDP, listen-only, same parser and sentence set as the
  UART path (D-014). Binds `0.0.0.0:<listen_port>`, not the manifest's
  named interface: `SO_BINDTODEVICE` needs `CAP_NET_RAW`/`CAP_NET_ADMIN`,
  the wrong trade for a listen-only sensor input. `source_ip` pinning is
  enforced at the socket, dropping any datagram from an unpinned source
  before it reaches the parser; an unpinned bus caps at enrichment.
- CRSF over UART from an RC receiver (RC channels, link statistics).
  Kill switch, takeover switch, surge/yaw sticks per D-025.
- Actuator command out: one `$CXOUT,<us0>,<us1>,...*HH` line per 100 ms
  control tick over UART, one integer-microsecond field per manifest-
  declared effector channel (D-026/D-027). Full contract is the module doc
  comment on coxswain-drivers/src/actuator_serial.rs; the far end must fail
  safe on silence (recommended: calibrated zero/center after 500 ms without
  a valid line). Allocation (tau to per-effector output) and the physical-
  to-microsecond rendering both run at the conn node from manifest
  calibration, so the far end carries no vessel knowledge: it copies fields
  to PWM channels and watches the line for silence, nothing more.
- Power report in, same UART, reverse direction: the far end sends
  `$CXPWR,<voltage_v>*HH`, recommended 1 Hz, from an INA2xx-class monitor
  on the actuator MCU. Command-then-report lite ahead of Cyphal
  (D-021, D-010); the parser ignores any other traffic on the wire
  (an echoed `$CXOUT`, say), so nothing further is required of the far
  end beyond emitting the line.
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
3. A far-end actuator controller: any small MCU that parses `$CXOUT` and
   drives the vessel's ESC/servo hardware from the microseconds it
   carries, failing safe on silence. This is the one piece of custom
   far-end firmware in the bring-up path; the Phase 9 actuator node
   replaces it.
4. Power monitoring reaching the failsafe matrix: the actuator far end
   measures the battery (INA2xx-class monitor) and reports
   `$CXPWR,<voltage_v>*HH` at about 1 Hz back on the actuator serial
   link. The input path exists; the far-end firmware owns the reading.
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
- Closed: power monitoring input path in real-serial mode. `$CXPWR`
  reports on the actuator link's reverse direction now feed the failsafe
  matrix (coxswain-drivers/src/actuator_serial.rs); the healthy default
  applies only until the first report arrives. Report staleness (what
  happens if reports stop) is not handled and remains an open item.
- Closed: the actuator serial link is a manifest `actuator_uart` bus now
  (D-026/D-027), mapped via `--port <bus_id>=<device>` like any other bus;
  an unmapped bus carrying effectors is a boot error (self-sufficiency,
  D-009). RC remains the one CLI option, not a manifest bus; promoting it
  into the schema is an open item recorded in the code.
- Baud for the GNSS bus comes from the manifest bus entry; confirm the
  chosen compass actually speaks its rated sentences at that baud with
  heading at 5 Hz or better.

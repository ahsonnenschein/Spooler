# SPOOLER

Wire tension controller firmware for a wire EDM (Electrical Discharge Machining) machine, running on the [W6100-EVB-Pico2](https://docs.wiznet.io/Product/iEthernet/W6100/w6100-evb-pico2) board (RP2350 + W6100 Ethernet).

The firmware maintains constant wire tension by controlling two stepper motors — a feed spool and a recovery spool — using a closed-loop PID controller. A spring-loaded dancer assembly measures wire tension via a linear potentiometer. The system communicates with a [LinuxCNC](https://linuxcnc.org/) host over Modbus TCP.

## How It Works

A thin brass wire (0.25mm) feeds from a supply spool, passes through the cutting zone, and is collected on a recovery spool. A spring-loaded dancer rides the wire between the two spools. If tension increases, the dancer moves one way; if it decreases, the other. The firmware keeps the dancer at a fixed setpoint by adjusting the recovery spool speed relative to the feed spool.

If the wire breaks, tension collapses and the dancer hits its minimum position — the firmware detects this and stops both motors immediately.

## Features

- Modbus TCP server on port 502 (DHCP, W6100 Ethernet)
- PIO-based stepper motor control — precise step timing independent of CPU load
- 100Hz PID control loop for dancer position
- Wire break detection with latching fault
- Auto/Manual mode via front-panel switch
- Manual jog buttons for feed and recovery spools
- Water quality (TDS) sensor readback via Modbus
- All parameters tunable live over Modbus (PID gains, setpoint, limits)

## Hardware

| Component | Part |
|---|---|
| Controller board | W6100-EVB-Pico2 (RP2350 + W6100) |
| Stepper drivers | Adafruit TMC2209 (×2) |
| Dancer sensor | KTR2-R-75 linear potentiometer (75mm stroke) |
| Water sensor | Keyestudio KS0429 TDS Meter V1.0 |

## GPIO Assignment

| Function | GPIO | Physical Pin |
|---|---|---|
| Feed STEP | GP10 | 14 |
| Feed DIR | GP11 | 15 |
| Recovery STEP | GP15 | 20 |
| Recovery DIR | GP14 | 19 |
| Auto/Manual switch | GP12 | 16 |
| Manual Feed button | GP13 | 17 |
| Manual Recover button | GP9 | 12 |
| Dancer pot wiper | GP27 (ADC1) | 32 |
| TDS sensor output | GP26 (ADC0) | 31 |
| Ethernet (reserved) | GP16–GP21 | — |

## Modbus Register Map

**Holding Registers** (read/write, FC03/FC06/FC16):

| Address | Name | Units |
|---|---|---|
| 40001 | WIRE_FEED_RATE | rev/min × 100 (e.g. 100 = 1.00 rev/min) |
| 40002 | DANCER_SETPOINT | cm × 1000 (e.g. 3750 = 37.5mm) |
| 40003 | MIN_DANCER_POS | cm × 1000 |
| 40004 | MAX_DANCER_POS | cm × 1000 |
| 40005 | PID_KP | gain × 1000 |
| 40006 | PID_KI | gain × 1000 |
| 40007 | PID_KD | gain × 1000 |
| 40008 | FAULT_CLEAR | write 1 to clear fault |
| 40009 | SPOOL_RESET | write 1 to reset |

**Input Registers** (read only, FC04):

| Address | Name | Units |
|---|---|---|
| 30001 | DANCER_POSITION | cm × 1000 (filtered) |
| 30002 | WATER_TDS | raw ADC 0–4095 |
| 30003 | FAULT_CODE | 0=none, 1=wire break, 2=over-travel |

**Coils** (read/write, FC01/FC05):

| Address | Name |
|---|---|
| 00001 | WIRE_FEED (0=off, 1=running) |

**Discrete Inputs** (read only, FC02):

| Address | Name |
|---|---|
| 10001 | WIRE_FAULT |
| 10002 | AUTO_MODE |

## Motor Parameters

- Stepper: 17HS19-1684S-PG27 (1.8°, 27:1 gearbox)
- Belt reduction: 20T → 80T (4:1)
- Microstepping: 8×
- **Total: 172,800 steps/rev at the spool**

## Building

Requires the ARM cross-compiler toolchain (`arm-none-eabi-gcc`). The Pico SDK is fetched automatically by CMake.

```bash
mkdir build && cd build
cmake ..
make
```

Output: `build/spooler.uf2`

## Flashing

1. Hold BOOTSEL on the board and plug in USB
2. The board mounts as a USB drive
3. Copy `spooler.uf2` to the drive — the board reboots automatically

## Serial Output

Connect at 115200 baud over USB to see status:

```
========================================
SPOOLER Wire Tension Controller
W6100-EVB-Pico2 (RP2350)
========================================
Tension controller initialized.
  Steps/spool rev: 172800
  Default setpoint: 37.5 mm

========================================
DHCP IP Assigned!
IP Address: 192.168.1.x
Modbus TCP Server ready on port 502
========================================
```

## Dependencies

- [WIZnet ioLibrary_Driver](https://github.com/Wiznet/ioLibrary_Driver) (included as a git submodule)
- Raspberry Pi Pico SDK 2.1.0 (fetched automatically)

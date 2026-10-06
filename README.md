# Gaelforce ESP32 pod (ODOM V2)

Firmware for an ESP32-S3 sensor pod that works out where the robot is on the
field. It drives nothing: it sends its pose estimate over RS-485 to the Brain.

It combines two quadrature tracking-wheel encoders, a BNO085 IMU and four
TMF8821 distance sensors in a particle filter (`gflib`), and reports the result
100 times a second.

Board: ESP32-S3-DevKitC-1 with an **N8R8** module. Built with PlatformIO and
ESP-IDF.

## Pin map (V2 PCB)

These are the pins the V2 PCB is wired to, and what `include/pod_config.hpp`
and `tools/tof_test` are set to.

| Function | Signal | V2 GPIO | V1 GPIO | Changed |
|---|---|---|---|---|
| Vertical encoder | A | **40** | 1 | yes |
| Vertical encoder | B | **42** | 2 | yes |
| Horizontal encoder | A | **39** | 3 | yes |
| Horizontal encoder | B | **41** | 4 | yes |
| IMU (BNO085) | RX | 8 | 8 | no |
| RS-485 | TX (DI) | 17 | 17 | no |
| RS-485 | RX (RO) | 18 | 18 | no |
| RS-485 | DE + RE (tied) | **7** | 21 | yes |
| ToF I2C | SDA (all 4) | 16 | 16 | no |
| ToF I2C | SCL (all 4) | 15 | 15 | no |
| ToF EN | FRONT | **4** | 11 | yes |
| ToF EN | RIGHT | 12 | 12 | no |
| ToF EN | REAR | **1** | 14 | yes |
| ToF EN | LEFT | **2** | 13 | yes |

Free for future use: 3 (strapping pin), 5, 6, 9, 10, 11, 13, 14, 21, 38.

### Pins that must not be used

On an N8R8 module:

| GPIO | Reason |
|---|---|
| 0, 45, 46 | strapping pins |
| 19, 20 | native USB (flashing and serial monitor) |
| 26-32 | SPI flash |
| 33-37 | octal PSRAM, wired inside the module even though this project leaves PSRAM off |
| 43, 44 | console UART |
| 48 | on-board RGB LED |

This is why the encoders went to 39-42 and not 35-38: 35, 36 and 37 are the
PSRAM pins.

## Wiring

### Encoders (via SN74LVC245A level converter)

The encoders connect to the A side of the converter and the ESP32 to the B side
(DIR tied high, OE to GND, VCC to 3.3 V). Each signal passes straight across
(A1 to B1 and so on). The B pins are arranged so the wiring is easy, not in the
order the firmware names them:

| Converter pin | Encoder wire | ESP32 GPIO |
|---|---|---|
| B1 | Vertical **B** | 42 |
| B2 | Horizontal **B** | 41 |
| B3 | Vertical **A** | 40 |
| B4 | Horizontal **A** | 39 |

Tie unused converter inputs to GND. If an encoder counts backwards, flip its
`kVertReversed` / `kHorizReversed` flag in `pod_config.hpp` rather than
rewiring.

### IMU (BNO085, UART-RVC mode)

| IMU pin | Connects to |
|---|---|
| SDA (becomes the IMU's serial TX in UART mode) | GPIO8 |
| SCL | nothing |
| VIN | 3.3 V |
| GND | GND |

The **P0 jumper must be bridged** to select UART-RVC mode. Without it the IMU
starts in I2C mode and GPIO8 never receives anything.

### RS-485 link to the Brain (SP3485)

| SP3485 pin | Connects to |
|---|---|
| DI | GPIO17 |
| RO | GPIO18 |
| DE and RE, tied together | GPIO7 |
| VCC / GND | 3.3 V / GND |
| A, B | the Brain's A, B |

Run a **ground wire** to the Brain alongside A and B. If the link only receives
garbage, A and B are probably swapped. DE is the talk/listen switch: the ESP32
switches it automatically around each message, so it must not be shared with
anything else.

### ToF sensors (4x TMF8821)

All four share SDA and SCL, are powered from **3.3 V**, and each has its own EN
pin because they all start at the same I2C address and are brought up one at a
time. Leave INT, GPIO0 and GPIO1 on the sensor boards unconnected.

The breakouts' own pull-up resistors are the only pull-ups on the bus, so at
least one breakout's I2C pull-up jumper must be closed.

## Changing a pin

Every pin lives in `include/pod_config.hpp`. The ToF bench test keeps its own
copy of the ToF pins in `tools/tof_test/src/main.cpp`, so change both. See
[PIN_CHANGES.md](PIN_CHANGES.md) for the history of each move.

## Bench test for the ToF sensors

`tools/tof_test` is a standalone project that brings up the four sensors with
the pod's own driver and prints one distance per sensor. It does not need
`gflib`.

```bash
pio run -d tools/tof_test -t upload -t monitor
```

It prints `FRONT`, `RIGHT`, `REAR` and `LEFT` distances in mm. `--` means
nothing is in range, `DEAD` means a sensor failed to start, and `STALLED` means
it stopped sending results. A reading that sticks at 20-60 mm and never changes
means something is right against the lens.

## Building the pod firmware

The pod firmware needs `gflib` (particle filter, odometry and link framing)
cloned next to this repo at `../gflib`; it is not in this repository.

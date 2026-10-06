# Pin changes (PCB rework)

Pin reassignments made in the PCB editor. They have been applied to
`include/pod_config.hpp` and the copy in `tools/tof_test/src/main.cpp`; the
"Firmware change needed" snippets below show what was changed.

Board: ESP32-S3-DevKitC-1, **N8R8** module. GPIO33-37 are wired to the octal
PSRAM inside the module and must not be used.

## Encoders

Moved from GPIO1-4 to GPIO39-42, four adjacent pins on header J3 (pins 6-9).

The converter (SN74LVC245A) is wired so B1/B2 carry the encoders' **B**
channels and B3/B4 carry their **A** channels, to make the wiring easier.
Each converter pin passes straight across to its partner (A1 <-> B1, etc.).

| Converter pin | Encoder wire     | ESP32 GPIO | Header pin | Old GPIO |
|---------------|------------------|------------|------------|----------|
| B1            | Vertical **B**   | 42         | J3_6       | 2        |
| B2            | Horizontal **B** | 41         | J3_7       | 4        |
| B3            | Vertical **A**   | 40         | J3_8       | 1        |
| B4            | Horizontal **A** | 39         | J3_9       | 3        |

Assumes DIR is tied high (A side -> B side): encoders on A1-A4, ESP32 on B1-B4.
If DIR is tied low, swap the sides.

### Firmware change needed

```cpp
constexpr int kVertEncAPin  = 40;
constexpr int kVertEncBPin  = 42;
constexpr int kHorizEncAPin = 39;
constexpr int kHorizEncBPin = 41;
```

If an encoder counts backwards after this, its A and B are swapped: flip
`kVertReversed` / `kHorizReversed` rather than rewiring.

## ToF sensors

FRONT, REAR and LEFT sensor EN moved to GPIO4, GPIO1 and GPIO2, which the
encoders freed up. GPIO11, GPIO13 and GPIO14 are now unused. (GPIO21 was
considered for LEFT but was RS-485 DE at the time; GPIO19 is native USB D-.)

| Signal            | Old GPIO | New GPIO |
|-------------------|----------|----------|
| EN, FRONT (tof0)  | 11       | 4        |
| EN, REAR (tof2)   | 14       | 1        |
| EN, LEFT (tof3)   | 13       | 2        |

### Firmware change needed

`include/pod_config.hpp` and `tools/tof_test/src/main.cpp`:

```cpp
constexpr int kTofEnPins[4] = {4, 12, 1, 2};
```

## RS-485 link

DE (tied to RE on the SP3485) moved to GPIO7 on the left header (J1), with
TX/RX on 17/18. It is the nearest free non-strapping pin: 15/16 between them
are I2C. GPIO21 is now unused. TX and RX are unchanged.

| Signal                | Old GPIO | New GPIO |
|-----------------------|----------|----------|
| RS-485 DE/RE (SP3485) | 21       | 7        |

### Firmware change needed

`include/pod_config.hpp`:

```cpp
constexpr int kRs485DePin = 7;
```

## Unchanged

| Signal              | GPIO               |
|---------------------|--------------------|
| IMU RX (BNO085)     | 8                  |
| RS-485 TX / RX      | 17 / 18            |
| I2C SDA / SCL       | 16 / 15            |
| ToF EN RIGHT        | 12                 |

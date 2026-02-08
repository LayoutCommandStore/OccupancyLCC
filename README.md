# OccupancyLCC
LCC Occupancy Detector — Raspberry Pi Pico + MCP2515

An NMRA LCC / OpenLCB node that detects track occupancy using 1–2 current-transformer (CT) coils and produces standard LCC events on CAN bus.

-----

## Hardware

### Bill of Materials

|Qty|Part                           |Notes                                        |
|---|-------------------------------|---------------------------------------------|
|1  |Raspberry Pi Pico              |RP2040-based board                           |
|1  |MCP2515 CAN module             |8 MHz crystal, TJA1050 or MCP2551 transceiver|
|1–2|CT coil (e.g. YHDC SCT-013-020)|20 A split-core current transformer          |
|1–2|33 Ω burden resistor           |Value depends on CT ratio — see below        |
|2–4|10 kΩ resistor                 |Bias voltage divider                         |
|1–2|10 µF electrolytic capacitor   |Smoothing on bias rail (optional)            |
|1  |120 Ω resistor                 |CAN bus termination (if end-of-bus)          |

### Wiring

```
                    Pico                         MCP2515 Module
              ┌──────────────┐               ┌──────────────────┐
              │          GP2 │──── SCK ──────│ SCK              │
              │          GP3 │──── MOSI ─────│ SI               │
              │          GP4 │──── MISO ─────│ SO               │
              │          GP5 │──── CS ───────│ CS               │
              │          GP6 │──── INT ──────│ INT              │
              │         3V3  │───────────────│ VCC              │
              │         GND  │───────────────│ GND              │
              └──────────────┘               │          CANH ───│──── LCC CAN Bus
                                             │          CANL ───│──── LCC CAN Bus
                                             └──────────────────┘

        CT Coil Bias & ADC Circuit (repeat for each zone)
        ─────────────────────────────────────────────────
                         3.3 V
                           │
                          [10 kΩ]
                           │
        CT coil ───┬───[33 Ω burden]───┬─── GP26 (Zone 1)  or  GP27 (Zone 2)
                   │                   │
                  GND                 [10 kΩ]
                                       │
                                      GND

        The two 10 kΩ resistors form a voltage divider producing ≈ 1.65 V DC
        bias so the AC signal from the CT rides in the middle of the ADC range.
```

### CT Coil Burden Resistor Selection

The burden resistor converts the CT secondary current into a voltage. For a 1000:1 CT (like SCT-013-020 in voltage-output mode the burden is built in). For a current-output CT, choose R_burden so that at maximum expected track current the peak voltage stays below 1.65 V:

```
R_burden = (1.65 V × CT_ratio) / (I_max × √2)
```

For HO scale DCC (≈ 1–3 A max), a 33–100 Ω resistor works well with a 1000:1 CT.

-----

## Software Setup

### Arduino IDE

1. **Install the Arduino-Pico core** — follow [earlephilhower/arduino-pico](https://github.com/earlephilhower/arduino-pico) instructions.
1. **Install libraries** via Library Manager:
- `ACAN2515` by Pierre Molinaro
- `OpenLCB` (or `OpenLCBSingleThread`) — search “OpenLCB” or install from [github.com/openlcb](https://github.com/openlcb)
1. **Select board** → *Raspberry Pi Pico*.
1. Open `LCC_OccupancyDetector.ino` and upload.

### PlatformIO

```ini
[env:pico]
platform  = raspberrypi
board     = pico
framework = arduino
lib_deps  =
    pierremolinaro/ACAN2515
    ; Install OpenLCB from GitHub:
    ; https://github.com/openlcb/OpenLCB_Single_Thread
monitor_speed = 115200
```

-----

## Configuration

### Unique Node ID

Every LCC node must have a **globally unique 48-bit Node ID**. Edit the `NODE_ID` macro in the sketch. For personal use, simply change the last two bytes for each board you build:

```cpp
#define NODE_ID  {0x05, 0x02, 0x01, 0x01, 0x00, 0x02}  // board #2
```

For production use, request your own range from [OpenLCB Unique ID registry](https://registry.openlcb.org).

### CDI Configuration via JMRI

Once the node is on the bus, open **JMRI → LCC → Configure Nodes** and you will see:

|Parameter         |Description                                              |Default   |
|------------------|---------------------------------------------------------|----------|
|**Zone Name**     |Friendly label                                           |*(empty)* |
|**Threshold**     |ADC peak-to-peak value that indicates occupancy (10–4000)|100       |
|**Debounce**      |Consecutive 50 ms ticks before state changes (1–20)      |4 (200 ms)|
|**Enabled**       |0 = disabled, 1 = enabled                                |1         |
|**Occupied Event**|Event ID produced when zone becomes occupied             |auto      |
|**Clear Event**   |Event ID produced when zone becomes clear                |auto      |

### Tuning the Threshold

1. With **no trains** on the track, open Serial Monitor (115200 baud) and note the reported peak-to-peak values — this is your noise floor.
1. Place a **locomotive** (decoder on, motor off) on the track and note the new value.
1. Set the threshold midway between the noise floor and the idle-loco reading.
1. Typical starting values: **50–150** for HO DCC with a 1000:1 CT.

-----

## Events Produced

|Event Index|Meaning        |
|-----------|---------------|
|0          |Zone 1 Occupied|
|1          |Zone 1 Clear   |
|2          |Zone 2 Occupied|
|3          |Zone 2 Clear   |

Default Event IDs are derived from the Node ID. They can be changed via CDI/JMRI to match your layout’s event scheme.

-----

## LED Behaviour

|Pattern            |Meaning                        |
|-------------------|-------------------------------|
|Slow blink (1 Hz)  |All zones clear                |
|Fast blink (4 Hz)  |At least one zone occupied     |
|Rapid toggle (5 Hz)|CAN init failure — check wiring|

-----

## Theory of Operation

1. Every **50 ms** the firmware samples each enabled CT coil’s ADC input over a 20 ms window, recording the minimum and maximum readings.
1. The **peak-to-peak** difference is compared against the configured threshold.
1. A **debounce counter** increments each scan period that the raw state differs from the current logical state. When the counter reaches the configured debounce value, the logical state flips and the appropriate LCC event is produced.
1. The OpenLCB stack handles **alias negotiation**, **CDI serving**, **event identification**, and all other LCC protocol requirements automatically.

-----

## Troubleshooting

|Symptom                          |Check                                                                                                  |
|---------------------------------|-------------------------------------------------------------------------------------------------------|
|No node appears on bus           |CAN wiring, 120 Ω termination, crystal frequency matches `ACAN2515Settings` constructor (8 MHz default)|
|False occupancy                  |Lower `Threshold` or increase `Debounce`; check for electrical noise on ADC lines                      |
|Never detects trains             |Raise `Threshold`; verify CT coil is around the correct wire; check burden resistor value              |
|Serial shows “MCP2515 init error”|SPI wiring, CS pin, MCP2515 module power                                                               |

-----

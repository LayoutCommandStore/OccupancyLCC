// =============================================================================
// LCC Occupancy Detector — Raspberry Pi Pico + MCP2515 CAN Transceiver
// =============================================================================
// Implements an NMRA LCC (OpenLCB) node that monitors 1–2 current-transformer
// (CT) coils and produces Occupied / Clear events for each detection zone.
//
// Hardware assumptions
//   • Raspberry Pi Pico (RP2040) running Arduino-Pico core
//   • MCP2515 CAN controller on SPI0  (CS=GP5, INT=GP6, SCK=GP2, MOSI=GP3, MISO=GP4)
//   • CT coil 1 burden resistor centred at 1.65 V → ADC on GP26 (A0)
//   • CT coil 2 burden resistor centred at 1.65 V → ADC on GP27 (A1)
//   • Optional: on-board LED on GP25 for heartbeat / activity
//
// Library dependencies (install via Library Manager or manually):
//   • OpenLCBSingleThread  (OpenLCB single-thread Arduino library)
//   • ACAN2515             (MCP2515 CAN driver by Pierre Molinaro)
//   • EEPROM               (built-in with Arduino-Pico core)
//
// SPDX-License-Identifier: GPL-2.0-or-later
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

// ---- OpenLCB / LCC single-thread library headers ---------------------------
#include <OpenLCBMid.h>          // pulls in OlcbCommonCAN, EventTable, etc.

// ---- MCP2515 CAN driver ----------------------------------------------------
#include <ACAN2515.h>

// =============================================================================
//  PIN DEFINITIONS
// =============================================================================
static const uint8_t PIN_CAN_CS   = 5;
static const uint8_t PIN_CAN_INT  = 6;
static const uint8_t PIN_CAN_SCK  = 2;
static const uint8_t PIN_CAN_MOSI = 3;
static const uint8_t PIN_CAN_MISO = 4;

static const uint8_t PIN_CT1      = 26;   // ADC0 — CT coil 1
static const uint8_t PIN_CT2      = 27;   // ADC1 — CT coil 2

static const uint8_t PIN_LED      = 25;   // on-board LED

// =============================================================================
//  NODE IDENTITY  (change bytes 4-5 per board to make each node unique)
// =============================================================================
//  Default Node ID:  05.02.01.01.00.01
//  You MUST obtain your own unique ID range from OpenLCB if you manufacture
//  boards.  For personal/hobby use change the last two bytes per node.
#define NODE_ID  {0x05, 0x02, 0x01, 0x01, 0x00, 0x01}

// =============================================================================
//  EVENT TABLE
//  4 produced events (no consumed events):
//    Index 0 — Zone 1 Occupied
//    Index 1 — Zone 1 Clear
//    Index 2 — Zone 2 Occupied
//    Index 3 — Zone 2 Clear
// =============================================================================
#define NUM_EVENTS  4

// Starting default Event IDs — same prefix as Node ID with sequential suffixes
#define EVENT_BASE  {0x05, 0x02, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00}

// Event roles — all are producers
static const uint8_t eventFlags[NUM_EVENTS] = {
     OLLCB_EF_PRODUCER,   // Zone 1 Occupied
    OLLCB_EF_PRODUCER,   // Zone 1 Clear
    OLLCB_EF_PRODUCER,   // Zone 2 Occupied
    OLLCB_EF_PRODUCER,   // Zone 2 Clear
};

// =============================================================================
//  EEPROM / NODE-MEMORY LAYOUT
//  The OpenLCB library stores event IDs and user configuration in EEPROM.
//  We define offsets manually for clarity.
// =============================================================================
//  Byte 0          : reset marker (written by library)
//  Bytes 1..6      : Node ID
//  Bytes 7..70     : Event IDs (8 bytes × NUM_EVENTS = 32 bytes, padded to 64)
//  Bytes 71..      : User configuration (see configLayout)
// =============================================================================

// Per-zone user-config offsets *relative to the start of user config area*
// Zone 1
#define CFG_ZONE1_THRESHOLD_OFFSET   0   // uint16  (ADC threshold, default 100)
#define CFG_ZONE1_DEBOUNCE_OFFSET    2   // uint8   (debounce in 50ms ticks, default 4 = 200ms)
#define CFG_ZONE1_ENABLED_OFFSET     3   // uint8   (0=disabled, 1=enabled, default 1)
#define CFG_ZONE1_NAME_OFFSET        4   // string  (user-friendly name, 32 chars)
// Zone 2
#define CFG_ZONE2_THRESHOLD_OFFSET  36
#define CFG_ZONE2_DEBOUNCE_OFFSET   38
#define CFG_ZONE2_ENABLED_OFFSET    39
#define CFG_ZONE2_NAME_OFFSET       40

#define CFG_NODE_NAME_OFFSET        72   // 32-byte user node name
#define CFG_NODE_DESC_OFFSET       104   // 64-byte user node description

#define USER_CONFIG_SIZE           168

// Total EEPROM needed
#define EEPROM_NEEDED  (64 + NUM_EVENTS * 8 + USER_CONFIG_SIZE + 16 /* padding */)

// =============================================================================
//  CDI — Configuration Description Information (XML)
//  This is served to JMRI / LCC configuration tools over the bus.
// =============================================================================
const char cdi[] PROGMEM = R"xmldata(<?xml version='1.0'?>
<cdi xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'
     xsi:noNamespaceSchemaLocation='https://openlcb.org/schema/cdi/1/1/cdi.xsd'>

  <identification>
    <manufacturer>Hobby DIY</manufacturer>
    <model>PicoOccupancy2Z</model>
    <hardwareVersion>1.0</hardwareVersion>
    <softwareVersion>1.0.0</softwareVersion>
  </identification>

  <acdi/>

  <segment space='253' origin='0'>
    <name>User Identification</name>
    <string size='32'>
      <name>Node Name</name>
      <description>A short name for this node.</description>
    </string>
    <string size='64'>
      <name>Node Description</name>
      <description>A longer description of this node and its location.</description>
    </string>
  </segment>

  <segment space='251' origin='0'>
    <name>Occupancy Detectors</name>

    <group replication='2'>
      <name>Zone</name>
      <repname>Zone</repname>

      <string size='32'>
        <name>Zone Name</name>
        <description>Friendly name for this detection zone.</description>
      </string>

      <int size='2'>
        <name>Threshold</name>
        <description>ADC peak-to-peak threshold above which the zone is
considered occupied.  Range 10–4000.  Start with 100 and
adjust downward for more sensitivity.</description>
        <min>10</min>
        <max>4000</max>
        <default>100</default>
      </int>

      <int size='1'>
        <name>Debounce (× 50 ms)</name>
        <description>Number of 50 ms periods the signal must remain
above/below threshold before the state changes.
Range 1–20.  Default 4 (200 ms).</description>
        <min>1</min>
        <max>20</max>
        <default>4</default>
      </int>

      <int size='1'>
        <name>Enabled</name>
        <description>Set to 1 to enable this zone, 0 to disable.</description>
        <min>0</min>
        <max>1</max>
        <default>1</default>
        <map>
          <relation><property>0</property><value>Disabled</value></relation>
          <relation><property>1</property><value>Enabled</value></relation>
        </map>
      </int>

      <eventid>
        <name>Occupied Event</name>
        <description>Event produced when current is detected (zone occupied).</description>
      </eventid>

      <eventid>
        <name>Clear Event</name>
        <description>Event produced when current drops (zone clear).</description>
      </eventid>

    </group>
  </segment>

</cdi>
)xmldata";

// =============================================================================
//  GLOBAL OBJECTS
// =============================================================================
ACAN2515 canDriver(PIN_CAN_CS, SPI, PIN_CAN_INT);

// OpenLCB node memory image
static uint8_t nodeIdBytes[] = NODE_ID;

// Event ID storage — initialised from EEPROM by the library
static EventID eventIds[NUM_EVENTS];

// =============================================================================
//  OCCUPANCY DETECTOR STATE
// =============================================================================
struct ZoneState {
    bool     occupied;          // current logical state
    uint8_t  debounceCount;     // ticks in the pending-change direction
    uint16_t lastPeakToPeak;    // most recent measured p-p value
};

static ZoneState zones[2] = { {false,0,0}, {false,0,0} };

// Per-zone cached config (read from EEPROM at boot / on config write)
struct ZoneConfig {
    uint16_t threshold;
    uint8_t  debounce;
    bool     enabled;
};

static ZoneConfig zoneConfigs[2];

// =============================================================================
//  FORWARD DECLARATIONS
// =============================================================================
void loadZoneConfig();
uint16_t measurePeakToPeak(uint8_t pin, uint16_t windowMs);
void processZone(uint8_t zoneIndex, uint8_t adcPin);
void produceOccupiedEvent(uint8_t zone);
void produceClearEvent(uint8_t zone);

// =============================================================================
//  OpenLCB CALLBACKS
// =============================================================================

// Called by library when a configuration value is written via LCC
void userConfigWritten(uint16_t address, uint16_t length) {
    // Reload our cached configuration
    loadZoneConfig();
}

// Called by library to let us produce events from inputs
// This is invoked every loop iteration
void produceFromInputs() {
    // We handle timing ourselves in loop(); nothing needed here.
}

// Called by library on factory reset to set default event IDs
void userInitAll() {
    uint8_t base[] = EVENT_BASE;
    for (uint8_t i = 0; i < NUM_EVENTS; i++) {
        memcpy(&eventIds[i], base, 8);
        eventIds[i].val[7] = i + 1;   // sequential suffix
    }
}

// Called by library to determine current state when a consumer queries us
// Return 0 = invalid/unknown, 1 = valid-active, 2 = valid-inactive
uint8_t queryProducerState(uint16_t index) {
    uint8_t zone  = index / 2;
    bool isOccupiedEvent = (index % 2) == 0;
    if (zone >= 2) return 0;
    if (!zoneConfigs[zone].enabled) return 0;
    bool occupied = zones[zone].occupied;
    if (isOccupiedEvent)
        return occupied ? 1 : 2;   // active if occupied
    else
        return occupied ? 2 : 1;   // active if clear
}

// =============================================================================
//  CAN DRIVER ADAPTER
//  The OpenLCB library expects a thin CAN interface.  We bridge ACAN2515.
// =============================================================================

// Transmit buffer
static CANMessage txMsg;
static CANMessage rxMsg;

bool canAvailable() {
    return canDriver.available();
}

bool canReceive(CANMessage *msg) {
    return canDriver.receive(*msg);
}

bool canSend(CANMessage *msg) {
    return canDriver.tryToSend(*msg);
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    // --- Serial for debug ---------------------------------------------------
    Serial.begin(115200);
    delay(500);
    Serial.println(F("LCC Occupancy Detector — Pico + MCP2515"));

    // --- LED ----------------------------------------------------------------
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // --- ADC ----------------------------------------------------------------
    analogReadResolution(12);   // 0–4095 on RP2040
    pinMode(PIN_CT1, INPUT);
    pinMode(PIN_CT2, INPUT);

    // --- SPI & MCP2515 ------------------------------------------------------
    SPI.setSCK(PIN_CAN_SCK);
    SPI.setTX(PIN_CAN_MOSI);
    SPI.setRX(PIN_CAN_MISO);
    SPI.begin();

    // Configure MCP2515 — 8 MHz crystal, 125 kbps CAN (LCC standard bit rate)
    ACAN2515Settings canSettings(8UL * 1000UL * 1000UL, 125UL * 1000UL);
    canSettings.mRequestedMode = ACAN2515Settings::NormalMode;

    const uint16_t errCode = canDriver.begin(canSettings, [] { canDriver.isr(); });
    if (errCode != 0) {
        Serial.print(F("MCP2515 init error 0x"));
        Serial.println(errCode, HEX);
        while (true) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            delay(200);
        }
    }
    Serial.println(F("MCP2515 initialised — 125 kbps"));

    // --- EEPROM -------------------------------------------------------------
    EEPROM.begin(EEPROM_NEEDED);

    // --- OpenLCB node initialisation ----------------------------------------
    // (The library's Olcb_init handles alias negotiation, memory setup, etc.)
    Olcb_init(nodeIdBytes, eventIds, eventFlags, NUM_EVENTS, cdi, sizeof(cdi));

    // --- Load zone configuration from EEPROM --------------------------------
    loadZoneConfig();

    Serial.println(F("Node started."));
}

// =============================================================================
//  LOOP
// =============================================================================
static uint32_t lastScanMs  = 0;
static uint32_t lastBlinkMs = 0;
static bool     ledState    = false;

void loop() {
    // --- Feed the OpenLCB stack (alias management, datagram handling, etc.) --
    Olcb_process();

    // --- Occupancy scan every 50 ms -----------------------------------------
    uint32_t now = millis();
    if (now - lastScanMs >= 50) {
        lastScanMs = now;
        processZone(0, PIN_CT1);
        processZone(1, PIN_CT2);
    }

    // --- Heartbeat LED — blink pattern indicates any-zone-occupied ----------
    if (now - lastBlinkMs >= (zones[0].occupied || zones[1].occupied ? 250 : 1000)) {
        lastBlinkMs = now;
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState);
    }
}

// =============================================================================
//  OCCUPANCY MEASUREMENT
// =============================================================================

// Measure peak-to-peak ADC swing over a short window.
// CT coil output is AC riding on a DC bias (~1.65 V).  We find the
// difference between the highest and lowest samples; if it exceeds the
// threshold the zone is occupied.
uint16_t measurePeakToPeak(uint8_t pin, uint16_t windowMs) {
    uint16_t sigMin = 4095;
    uint16_t sigMax = 0;
    uint32_t t0 = millis();

    while ((millis() - t0) < windowMs) {
        uint16_t sample = analogRead(pin);
        if (sample > sigMax) sigMax = sample;
        if (sample < sigMin) sigMin = sample;
    }
    return (sigMax >= sigMin) ? (sigMax - sigMin) : 0;
}

void processZone(uint8_t zoneIndex, uint8_t adcPin) {
    ZoneConfig &cfg = zoneConfigs[zoneIndex];
    ZoneState  &st  = zones[zoneIndex];

    if (!cfg.enabled) {
        // If zone was occupied when disabled, send a clear event
        if (st.occupied) {
            st.occupied = false;
            st.debounceCount = 0;
            produceClearEvent(zoneIndex);
        }
        return;
    }

    // Measure over a 20 ms window (~1 full cycle at 50 Hz, >1 at 60 Hz)
    uint16_t pp = measurePeakToPeak(adcPin, 20);
    st.lastPeakToPeak = pp;

    bool rawOccupied = (pp >= cfg.threshold);

    if (rawOccupied != st.occupied) {
        st.debounceCount++;
        if (st.debounceCount >= cfg.debounce) {
            st.occupied = rawOccupied;
            st.debounceCount = 0;
            if (st.occupied) {
                produceOccupiedEvent(zoneIndex);
            } else {
                produceClearEvent(zoneIndex);
            }
        }
    } else {
        st.debounceCount = 0;
    }
}

// =============================================================================
//  EVENT PRODUCTION HELPERS
// =============================================================================

void produceOccupiedEvent(uint8_t zone) {
    uint8_t idx = zone * 2;       // event index 0 or 2
    Serial.print(F("Zone "));
    Serial.print(zone + 1);
    Serial.println(F(" → OCCUPIED"));
    OpenLcb_produceEvent(idx);
}

void produceClearEvent(uint8_t zone) {
    uint8_t idx = zone * 2 + 1;   // event index 1 or 3
    Serial.print(F("Zone "));
    Serial.print(zone + 1);
    Serial.println(F(" → CLEAR"));
    OpenLcb_produceEvent(idx);
}

// =============================================================================
//  CONFIGURATION HELPERS
// =============================================================================

// Read zone-specific settings from EEPROM into RAM cache
void loadZoneConfig() {
    // Zone 1
    EEPROM.get(CFG_ZONE1_THRESHOLD_OFFSET, zoneConfigs[0].threshold);
    zoneConfigs[0].debounce = EEPROM.read(CFG_ZONE1_DEBOUNCE_OFFSET);
    zoneConfigs[0].enabled  = EEPROM.read(CFG_ZONE1_ENABLED_OFFSET) != 0;

    // Zone 2
    EEPROM.get(CFG_ZONE2_THRESHOLD_OFFSET, zoneConfigs[1].threshold);
    zoneConfigs[1].debounce = EEPROM.read(CFG_ZONE2_DEBOUNCE_OFFSET);
    zoneConfigs[1].enabled  = EEPROM.read(CFG_ZONE2_ENABLED_OFFSET) != 0;

    // Sanity clamp
    for (uint8_t i = 0; i < 2; i++) {
        if (zoneConfigs[i].threshold < 10 || zoneConfigs[i].threshold > 4000)
            zoneConfigs[i].threshold = 100;
        if (zoneConfigs[i].debounce < 1 || zoneConfigs[i].debounce > 20)
            zoneConfigs[i].debounce = 4;
    }

    Serial.println(F("Config loaded:"));
    for (uint8_t i = 0; i < 2; i++) {
        Serial.print(F("  Zone "));  Serial.print(i + 1);
        Serial.print(F("  thr="));   Serial.print(zoneConfigs[i].threshold);
        Serial.print(F("  deb="));   Serial.print(zoneConfigs[i].debounce);
        Serial.print(F("  en="));    Serial.println(zoneConfigs[i].enabled);
    }
}

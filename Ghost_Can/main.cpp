/*
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_port = COM10

lib_deps = CAN

build_flags =
  -D CAN_RX_GPIO=16
  -D CAN_TX_GPIO=17

monitor_filters = esp32_exception_decoder
build_type = debug

*/

/* Ghost 192S BMS — Full parser & friendly summary
   - Parses groups 0x01..0x54 per the PDF you provided
   - Keeps ISR small: onReceive copies bytes and sets flag
   - All parsing happens in loop() and is organized by helper functions
   - Periodic summary every 1s, concise and human-friendly
   - Toggle DEBUG to print raw frames as they arrive
*/

#include <Arduino.h>
#include <CAN.h>

#ifndef CAN_RX_GPIO
#define CAN_RX_GPIO 16
#endif
#ifndef CAN_TX_GPIO
#define CAN_TX_GPIO 17
#endif

// USER TUNABLES
#define SUMMARY_PERIOD_MS 1000UL
#define DEBUG 0 // set to 1 to print raw frames as they arrive

// --- runtime flags + rx buffer (ISR-friendly) ---
volatile bool frameReceived = false;
uint8_t rxData[8];
uint32_t rxId = 0;
int rxLen = 0;

// connection state
bool connected = false;
unsigned long lastPrintMillis = 0;

// --- storage for parsed data ---
const int MAX_CELLS = 192;
float cellVoltages[MAX_CELLS];   // V; 0.0 => unused
int numCellsDetected = 0;

const int MAX_MODULE_TEMPS = 16;
float moduleTemps[MAX_MODULE_TEMPS]; // °C; very negative => unused

// Pack and status info (many fields default to invalid/sentinel)
float packTotalVoltage = NAN; // V (if group3 exists)
float packTotalCurrent = NAN; // A (signed)
float packTotalPower   = NAN; // W

uint32_t usedCapacity_Ah = 0;    // Group 4 used capacity
uint16_t socPercent = 0;         // Group 4 SOC %
uint32_t chargeCapacity_Ah = 0;  // Group 4

// Group1/2/5/6/80/81/82/83/84 fields (initialized to sentinel)
uint16_t dischargeProtect_V = 0;
uint16_t protectCurrent_A = 0;
uint16_t packCapacity_Ah = 0;

uint16_t stringCount = 0;
uint16_t chargeProtect_V = 0;
int16_t  protectionTemperature = 0;

uint16_t chargeRecover_V = 0;
uint16_t dischargeRecover_V = 0;
uint16_t remainingCapacity_Ah = 0;

int16_t hostTemperature = 0;
uint32_t statusAccountingRaw = 0; // raw bytes (for decoding flags)
uint16_t balanceStartVoltage = 0;

uint16_t lowVoltageProtect = 0;
uint16_t lowVoltageDelay = 0;
uint16_t numTriggeredCells = 0;

uint16_t balanceRefVoltage = 0;
uint16_t minCellVoltage_mV = 0;
uint16_t maxCellVoltage_mV = 0;
uint8_t mosStatusByte = 0; // bitmask

uint32_t accumTotalCapacity = 0;
uint16_t prechargeDelay = 0;
uint8_t lcdStatus = 0; // 0x01 on, 0x02 off

uint16_t diffPressureSetting = 0;
uint16_t autoResetSetting = 0;
int16_t lowTempProtectionValue = 0;
uint8_t protectionHistoryLogCode = 0;

uint16_t hallSensorType = 0;
uint16_t fanStartValue = 0;
uint16_t ptcStartValue = 0;
uint8_t defaultChannelState = 0; // 0x01 on, 0x02 off

// Protection history textual decode (Table 2)
const char* protectionHistoryDecode(uint8_t code) {
  switch (code) {
    case 0x01: return "Overcurrent protection";
    case 0x02: return "Over-discharge protection";
    case 0x03: return "Overcharge protection";
    case 0x04: return "Over temperature protection";
    case 0x05: return "Battery string error protection";
    case 0x06: return "Damaged charging relay";
    case 0x07: return "Damaged discharge relay";
    case 0x08: return "Low voltage power outage protection";
    case 0x09: return "Voltage difference protection";
    case 0x0A: return "Low temperature protection";
    case 0x00: return "No history";
    default:   return "Unknown history code";
  }
}

// --- helpers for printing nicely ---
String onOff(bool v) { return v ? "ON" : "OFF"; }
String mosStateStr(uint8_t mosByte) {
  // PDF: byte 8 of group 81: 0x01 DischargeMOS, 0x10 ChargeMOS
  bool discharge = mosByte & 0x01;
  bool charge = mosByte & 0x10;
  String s = "";
  if (discharge) s += "DischargeMOS ON ";
  else s += "DischargeMOS OFF ";
  if (charge) s += "| ChargeMOS ON";
  else s += "| ChargeMOS OFF";
  return s;
}

// decode status accounting (best-effort using Table 1)
void decodeStatusAccounting(uint32_t raw) {
  // Table 1 in the PDF was ambiguous about exact bit layout -> print helpful breakdown:
  Serial.print("  Status Accounting (raw hex): 0x"); Serial.println(raw, HEX);
  // We'll attempt to extract common markers (presentations may vary by vendor):
  // If bit field mapping is unknown, show bytes individually for quick diagnosis.
  Serial.print("   bytes: ");
  for (int b = 0; b < 4; ++b) {
    uint8_t v = (raw >> (8*(3-b))) & 0xFF;
    if (v < 0x10) Serial.print('0');
    Serial.print(v, HEX); Serial.print(' ');
  }
  Serial.println();
  // If vendor uses specific bits, user can expand here.
}

// compute cell min/max/delta
void computeCellStats(float &minV, int &minIdx, float &maxV, int &maxIdx, float &avg) {
  minV = 1e9; maxV = -1e9; avg = 0.0; int count = 0;
  minIdx = -1; maxIdx = -1;
  for (int i = 0; i < numCellsDetected; ++i) {
    float v = cellVoltages[i];
    if (v <= 0.0f) continue;
    if (v < minV) { minV = v; minIdx = i; }
    if (v > maxV) { maxV = v; maxIdx = i; }
    avg += v; count++;
  }
  if (count > 0) avg /= count;
  else { minV = 0.0; maxV = 0.0; avg = 0.0; }
}

// --- send handshake to BMS (same as your working code) ---
void sendHandshake() {
  uint32_t txId = 0xF4;

  // 0x1C CAN OK
  CAN.beginExtendedPacket(txId, 8);
  CAN.write(0x1C); CAN.write(0x00); CAN.write(0x02);
  for (int i = 0; i < 5; ++i) CAN.write(0x00);
  CAN.endPacket();
  Serial.println("Sent CAN OK (0x1C 0x0002)");

  delay(150);

  // 0x10 Connect
  CAN.beginExtendedPacket(txId, 8);
  CAN.write(0x10); CAN.write(0x00); CAN.write(0x02);
  for (int i = 0; i < 5; ++i) CAN.write(0x00);
  CAN.endPacket();
  Serial.println("Sent connect command (0x10 0x0002)");
}

// --- ISR (tiny) ---
void onReceive(int packetSize) {
  if (packetSize == 8) {
    rxId = CAN.packetId();
    rxLen = packetSize;
    for (int i = 0; i < packetSize; ++i) rxData[i] = CAN.read();
    frameReceived = true;
  } else {
    // flush if different size
    while (CAN.available()) CAN.read();
  }
}

// --- parsing helpers for groups ---
// Helper to read big-endian 16-bit from two bytes
static inline uint16_t be16(const uint8_t *ptr) {
  return (uint16_t(ptr[0]) << 8) | ptr[1];
}
// Helper to read signed 16 bit big-endian
static inline int16_t be16s(const uint8_t *ptr) {
  return (int16_t)((uint16_t(ptr[0]) << 8) | ptr[1]);
}

// Primary single-frame parser (executed in main loop context)
void parseFrameSafe(uint8_t *data, int len, uint32_t id) {
  if (id != 0xF5) return; // only parse from BMS send id
  connected = true;

  uint8_t header = data[0];

  if (DEBUG) {
    Serial.print("RAW FRAME 0x"); Serial.print(header, HEX); Serial.print(": ");
    for (int i = 0; i < len; ++i) {
      if (data[i] < 0x10) Serial.print('0');
      Serial.print(data[i], HEX); Serial.print(' ');
    }
    Serial.println();
  }

  // Cell voltage groups (0x07 .. 0x46) => groups 7..70
  if (header >= 0x07 && header <= 0x46) {
    int groupIndex = header - 0x07; // 0-based
    int firstCell = groupIndex * 3; // 0-based
    for (int i = 0; i < 3; ++i) {
      int bhi = 1 + i*2;
      int blo = 2 + i*2;
      uint16_t raw = be16(&data[bhi]);
      if (raw == 0) {
        cellVoltages[firstCell + i] = 0.0f;
      } else {
        // PDF indicates cells in mV
        cellVoltages[firstCell + i] = raw / 1000.0f;
        if ((firstCell + i + 1) > numCellsDetected && cellVoltages[firstCell + i] > 0.0f) {
          numCellsDetected = firstCell + i + 1;
          if (numCellsDetected > MAX_CELLS) numCellsDetected = MAX_CELLS;
        }
      }
    }
    return;
  }

  // Module temperatures groups 0x47..0x4F (71..79)
  if (header >= 0x47 && header <= 0x4F) {
    // The spec uses a varying layout: some bytes not used, and a "polarity" byte on some groups.
    // We attempt to parse module temps present in bytes (3-4), (6-7), depending on group layout.
    int baseModule = (header - 0x47) * 2; // approximate mapping (2 modules per group in many cases)
    // Many groups use bytes [3,4] and [6,7] for temps
    uint16_t m1 = be16(&data[3]);
    if (m1 != 0 && (baseModule) < MAX_MODULE_TEMPS) moduleTemps[baseModule] = (float)m1 / 10.0f;
    uint16_t m2 = be16(&data[6]);
    if (m2 != 0 && (baseModule + 1) < MAX_MODULE_TEMPS) moduleTemps[baseModule + 1] = (float)m2 / 10.0f;
    // Some groups include polarity bits in byte 2 — we ignore polarity for now and use absolute value.
    return;
  }

  // Group 1 (0x01): Discharge protection voltage, protective current, pack capacity
  if (header == 0x01) {
    dischargeProtect_V = be16(&data[1]); // probably in mV or in some units; PDF shows bytes only
    protectCurrent_A = be16(&data[3]);   // may need scaling; keep raw for now
    packCapacity_Ah = be16(&data[5]);
    return;
  }

  // Group 2 (0x02): Number of battery strings, Charge protection voltage, Protection temperature
  if (header == 0x02) {
    stringCount = be16(&data[1]);
    chargeProtect_V = be16(&data[3]);
    protectionTemperature = (int16_t)be16(&data[5]); // maybe °C * 10 or raw — keep raw
    return;
  }

  // Group 3 (0x03): total V/I/P (PDF: divide by 10)
  if (header == 0x03) {
    uint16_t v_raw = be16(&data[1]);
    int16_t i_raw = be16s(&data[3]); // treat as signed
    uint16_t p_raw = be16(&data[5]);
    packTotalVoltage = v_raw / 10.0f;
    packTotalCurrent = i_raw / 10.0f;
    packTotalPower = p_raw / 10.0f;
    return;
  }

  // Group 4 (0x04): used capacity, SOC %, charge capacity
  if (header == 0x04) {
    usedCapacity_Ah = be16(&data[1]);
    socPercent = be16(&data[3]);
    chargeCapacity_Ah = be16(&data[5]);
    return;
  }

  // Group 5 (0x05): charge recovery V, discharge recovery V, remaining capacity
  if (header == 0x05) {
    chargeRecover_V = be16(&data[1]);
    dischargeRecover_V = be16(&data[3]);
    remainingCapacity_Ah = be16(&data[5]);
    return;
  }

  // Group 6 (0x06): host temp, status accounting, balance start voltage
  if (header == 0x06) {
    hostTemperature = (int16_t)be16(&data[1]); // may require /10 depending on vendor
    // status accounting: bytes [3],[4] (and possibly others) -> store as 32-bit for inspection
    statusAccountingRaw = (uint32_t)be16(&data[3]);
    balanceStartVoltage = be16(&data[5]);
    return;
  }

  // Group 80 (0x50): low voltage protect, delay, num triggered cells
  if (header == 0x50) {
    lowVoltageProtect = be16(&data[1]);
    lowVoltageDelay = be16(&data[3]);
    numTriggeredCells = be16(&data[5]);
    return;
  }

  // Group 81 (0x51): balance ref voltage, min cell, max cell, MOS status
  if (header == 0x51) {
    balanceRefVoltage = be16(&data[1]);
    minCellVoltage_mV = be16(&data[3]);
    maxCellVoltage_mV = be16(&data[5]);
    mosStatusByte = data[7]; // last byte
    return;
  }

  // Group 82 (0x52): Accumulated capacity, precharge delay, LCD status
  if (header == 0x52) {
    // 4 bytes for accumulated total capacity (HighHigh..LowLow) (per PDF: 4 bytes)
    accumTotalCapacity = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 8) | data[4];
    prechargeDelay = be16(&data[5]);
    lcdStatus = data[7];
    return;
  }

  // Group 83 (0x53): differential pressure, auto reset setting, low temp protection, history log
  if (header == 0x53) {
    diffPressureSetting = be16(&data[1]);
    autoResetSetting = be16(&data[3]);
    lowTempProtectionValue = (int16_t)be16(&data[5]);
    protectionHistoryLogCode = data[7];
    return;
  }

  // Group 84 (0x54): hall sensor type, fan start, ptc start, default channel state
  if (header == 0x54) {
    hallSensorType = be16(&data[1]);
    fanStartValue = be16(&data[3]);
    ptcStartValue = be16(&data[5]);
    defaultChannelState = data[7];
    return;
  }

  // For any group we haven't specially handled, we keep quiet (or debug print)
  if (DEBUG) {
    Serial.print("Unhandled group 0x"); Serial.print(header, HEX); Serial.print(" raw: ");
    for (int i = 0; i < len; ++i) {
      if (data[i] < 0x10) Serial.print('0');
      Serial.print(data[i], HEX); Serial.print(' ');
    }
    Serial.println();
  }
}

// --- print a concise, clear summary every SUMMARY_PERIOD_MS ---
void printSummary() {
  Serial.println(F("====== Ghost BMS Summary ======"));
  Serial.print(F("Connected: ")); Serial.println(connected ? "YES" : "NO");
  Serial.println();

  // Pack overview
  Serial.println(F("---- Pack Overview ----"));
  if (!isnan(packTotalVoltage)) {
    Serial.print(" Total Voltage : "); Serial.print(packTotalVoltage, 2); Serial.println(" V");
  } else Serial.println(" Total Voltage : --");

  if (!isnan(packTotalCurrent)) {
    Serial.print(" Total Current : "); Serial.print(packTotalCurrent, 2); Serial.println(" A");
  } else Serial.println(" Total Current : --");

  if (!isnan(packTotalPower)) {
    Serial.print(" Total Power   : "); Serial.print(packTotalPower, 1); Serial.println(" W");
  } else Serial.println(" Total Power   : --");

  Serial.print(" Used Capacity : "); Serial.print(usedCapacity_Ah); Serial.println(" Ah");
  Serial.print(" Charge Cap    : "); Serial.print(chargeCapacity_Ah); Serial.println(" Ah");
  Serial.print(" Remaining Cap : "); Serial.print(remainingCapacity_Ah); Serial.println(" Ah");
  Serial.print(" SOC           : "); Serial.print(socPercent); Serial.println(" %");
  Serial.println();

  // Protection & status blocks
  Serial.println(F("---- Protections / Settings ----"));
  Serial.print(" Discharge protect V  : "); Serial.println(dischargeProtect_V);
  Serial.print(" Charge protect V     : "); Serial.println(chargeProtect_V);
  Serial.print(" Protect current      : "); Serial.println(protectCurrent_A);
  Serial.print(" Pack capacity (raw)  : "); Serial.println(packCapacity_Ah);
  Serial.print(" Host temperature     : "); Serial.println(hostTemperature);
  Serial.print(" Strings count        : "); Serial.println(stringCount);
  Serial.print(" Low-voltage prot     : "); Serial.print(lowVoltageProtect); Serial.print("  delay: "); Serial.println(lowVoltageDelay);
  Serial.print(" Num triggered cells  : "); Serial.println(numTriggeredCells);
  Serial.print(" Balance ref V        : "); Serial.println(balanceRefVoltage);
  Serial.print(" Min cell (mV)        : "); Serial.println(minCellVoltage_mV);
  Serial.print(" Max cell (mV)        : "); Serial.println(maxCellVoltage_mV);
  Serial.print(" MOS Status           : "); Serial.println(mosStateStr(mosStatusByte));
  Serial.print(" LCD status (0x)      : 0x"); Serial.println(lcdStatus, HEX);
  Serial.print(" Precharge delay      : "); Serial.println(prechargeDelay);
  Serial.print(" Accum total cap      : "); Serial.println(accumTotalCapacity);
  Serial.print(" Fan start value      : "); Serial.println(fanStartValue);
  Serial.print(" PTC start value      : "); Serial.println(ptcStartValue);
  Serial.print(" Default channel      : "); Serial.println(defaultChannelState == 0x01 ? "ON" : defaultChannelState == 0x02 ? "OFF" : "UNKNOWN");
  Serial.print(" Protection history   : "); Serial.println(protectionHistoryDecode(protectionHistoryLogCode));
  Serial.println();

  // Status accounting decode (best-effort)
  Serial.println(F("---- Status Accounting ----"));
  decodeStatusAccounting(statusAccountingRaw);
  Serial.println();

  // Cells summary
  Serial.println(F("---- Cells (first 48 shown) ----"));
  Serial.print(" Detected cells: "); Serial.println(numCellsDetected);
  float minV; int minIdx; float maxV; int maxIdx; float avgV;
  computeCellStats(minV, minIdx, maxV, maxIdx, avgV);
  if (numCellsDetected > 0) {
    Serial.print(" Min cell: C"); Serial.print(minIdx + 1); Serial.print(" = ");
    Serial.print(minV, 3); Serial.println(" V");
    Serial.print(" Max cell: C"); Serial.print(maxIdx + 1); Serial.print(" = ");
    Serial.print(maxV, 3); Serial.println(" V");
    Serial.print(" Delta    : "); Serial.print(maxV - minV, 3); Serial.println(" V");
    Serial.print(" Avg cell : "); Serial.print(avgV, 3); Serial.println(" V");
  }
  // print first 48 cells to keep summary short
  int toShow = min(numCellsDetected, 48);
  for (int i = 0; i < toShow; ++i) {
    Serial.print("C"); Serial.print(i+1); Serial.print(": ");
    if (cellVoltages[i] > 0.0f) Serial.print(cellVoltages[i], 3);
    else Serial.print("----");
    Serial.print("  ");
    if ((i+1) % 6 == 0) Serial.println();
  }
  Serial.println();
  if (numCellsDetected > 48) {
    Serial.print("... ("); Serial.print(numCellsDetected - 48); Serial.println(" more cells hidden)");
  }
  Serial.println();

  // Module temps
  Serial.println(F("---- Module Temperatures ----"));
  for (int i = 0; i < MAX_MODULE_TEMPS; ++i) {
    Serial.print("M"); Serial.print(i+1); Serial.print(": ");
    if (moduleTemps[i] > -200.0f) Serial.print(moduleTemps[i], 1);
    else Serial.print("--");
    Serial.print("  ");
    if ((i+1) % 8 == 0) Serial.println();
  }
  Serial.println("\n==============================\n");
}

// --- setup + loop ---
void setup() {
  Serial.begin(115200);
  delay(500);

  // initialize arrays
  for (int i = 0; i < MAX_CELLS; ++i) cellVoltages[i] = 0.0f;
  for (int i = 0; i < MAX_MODULE_TEMPS; ++i) moduleTemps[i] = -1000.0f; // sentinel

  CAN.setPins(CAN_RX_GPIO, CAN_TX_GPIO);
  if (!CAN.begin(250E3)) {
    Serial.println("CAN init failed!");
    while (1);
  }
  Serial.println("CAN bus initialized (250kbps, Extended frames)");
  CAN.onReceive(onReceive);

  // initial handshake
  sendHandshake();

  lastPrintMillis = millis();
}

void loop() {
  static unsigned long lastHandshake = 0;

  if (frameReceived) {
    frameReceived = false;
    parseFrameSafe(rxData, rxLen, rxId);
  }

  // retry handshake only until connected
  if (!connected && millis() - lastHandshake > 3000) {
    sendHandshake();
    lastHandshake = millis();
  }

  if (connected && millis() - lastPrintMillis >= SUMMARY_PERIOD_MS) {
    lastPrintMillis = millis();
    printSummary();
  }
}

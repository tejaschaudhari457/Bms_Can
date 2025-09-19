/* Ghost BMS improved listener
   - Sends 0x1C and 0x10 (both padded 8 bytes) until it sees 0xF5 frames
   - ISR is tiny; parsing/printing done in loop()
   - Parses:
     * Group 3 (total V, I, P) -- /10 as previous (kept for compatibility)
     * Group 4 (capacity, SOC) -- raw integers
     * Groups 7..70 -> cell voltages (3 cells per frame) 16-bit -> mV -> V
     * Groups 71..79 -> module temps (divide by 10 -> °C)
   - Adjust divisors if you want different units
*/

#include <Arduino.h>
#include <CAN.h>

#ifndef CAN_RX_GPIO
#define CAN_RX_GPIO 16
#endif
#ifndef CAN_TX_GPIO
#define CAN_TX_GPIO 17
#endif

volatile bool frameReceived = false;
uint8_t rxData[8];
uint32_t rxId = 0;
int rxLen = 0;

bool connected = false;
unsigned long lastPrintMillis = 0;

// storage
const int MAX_CELLS = 192;
float cellVoltages[MAX_CELLS]; // volts, 0 for unused
int numCellsDetected = 0;
float moduleTemps[32]; // generous size for module temps (index 0..)
/* NOTE: group 7 -> cells 1..3, group 8 -> cells 4..6, ... group 70 -> up to cell 192 */

void sendHandshake();
void onReceive(int packetSize);
void parseFrameSafe(uint8_t *data, int len, uint32_t id);
void printSummary();

void setup() {
  Serial.begin(115200);
  delay(1000);
  memset(cellVoltages, 0, sizeof(cellVoltages));
  memset(moduleTemps, 0xff, sizeof(moduleTemps));

  CAN.setPins(CAN_RX_GPIO, CAN_TX_GPIO);
  if (!CAN.begin(250E3)) {
    Serial.println("CAN init failed!");
    while (1);
  }
  Serial.println("CAN bus initialized");

  CAN.onReceive(onReceive);

  sendHandshake();
}

void loop() {
  static unsigned long lastHandshake = 0;

  if (frameReceived) {
    // clear flag quickly
    frameReceived = false;
    parseFrameSafe(rxData, rxLen, rxId);
  }

  // retry handshake only until connected
  if (!connected && millis() - lastHandshake > 3000) {
    sendHandshake();
    lastHandshake = millis();
  }

  // Periodic short summary every 1s when connected
  if (connected && millis() - lastPrintMillis >= 1000) {
    lastPrintMillis = millis();
    printSummary();
  }
}

// --- Handshake sends 0x1C then 0x10, both padded to 8 bytes ---
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

// --- Lightweight ISR: copy bytes and signal main loop ---
void onReceive(int packetSize) {
  if (packetSize == 8) {
    rxId = CAN.packetId();
    rxLen = packetSize;
    for (int i = 0; i < packetSize; i++) rxData[i] = CAN.read();
    frameReceived = true;
  } else {
    // flush
    while (CAN.available()) CAN.read();
  }
}

// --- Parsing happens in main loop context (safe) ---
void parseFrameSafe(uint8_t *data, int len, uint32_t id) {
  if (id != 0xF5) {
    // ignore other IDs
    return;
  }
  connected = true;

  uint8_t header = data[0];

  // Groups 7..70 => cell voltages (3 cells per frame)
  if (header >= 0x07 && header <= 0x46) {
    int groupIndex = header - 0x07; // 0-based
    int firstCell = groupIndex * 3; // 0-based cell index
    for (int i = 0; i < 3; ++i) {
      int bhi = 1 + i*2;
      int blo = 2 + i*2;
      uint16_t raw = (uint16_t(data[bhi]) << 8) | data[blo];
      if (raw == 0) {
        // unused cell slot
        cellVoltages[firstCell + i] = 0.0f;
      } else {
        // raw is in mV (observed). Convert to volts with 3 decimals
        cellVoltages[firstCell + i] = raw / 1000.0f;
        if ((firstCell + i + 1) > numCellsDetected && cellVoltages[firstCell + i] > 0.0f) {
          numCellsDetected = firstCell + i + 1;
        }
      }
    }
    // we won't print each cell frame here (keeps serial quieter)
    return;
  }

  // Module temperatures group 0x47..0x4F
  if (header >= 0x47 && header <= 0x4F) {
    // layout varies but observed: bytes 3-4 contain temp H/L for module N
    // We'll parse any non-zero 16-bit words in bytes [3,4], [6,7] etc as temps/10
    int baseModule = (header - 0x47) * 2; // modules per group varies in spec, approximate
    // Example observed: header 0x47 had bytes 3-4 = 0x01 0x17 -> 279 -> 27.9 C
    uint16_t m1 = (uint16_t(data[3]) << 8) | data[4];
    if (m1 != 0) moduleTemps[baseModule] = m1 / 10.0f;
    uint16_t m2 = (uint16_t(data[6]) << 8) | data[7];
    if (m2 != 0) moduleTemps[baseModule + 1] = m2 / 10.0f;
    return;
  }

  // Group 3: Total V/I/P — keep same scaling used before (divide by 10)
  if (header == 0x03) {
    float totalVoltage = ((data[1] << 8) | data[2]) / 10.0;
    float totalCurrent = ((data[3] << 8) | data[4]) / 10.0;
    float totalPower   = ((data[5] << 8) | data[6]) / 10.0;
    Serial.println("=== Group 3: Total Pack Info ===");
    Serial.print("  Total Voltage : "); Serial.print(totalVoltage); Serial.println(" V");
    Serial.print("  Total Current : "); Serial.print(totalCurrent); Serial.println(" A");
    Serial.print("  Total Power   : "); Serial.print(totalPower); Serial.println(" W");
    return;
  }

  // Group 4: SOC & capacities
  if (header == 0x04) {
    uint16_t usedCapacity = (data[1] << 8) | data[2];
    uint16_t socPercent   = (data[3] << 8) | data[4];
    uint16_t chargeCap    = (data[5] << 8) | data[6];
    Serial.println("=== Group 4: SOC & Capacity ===");
    Serial.print("  Used Capacity : "); Serial.print(usedCapacity); Serial.println(" Ah");
    Serial.print("  SOC Percent   : "); Serial.print(socPercent); Serial.println(" %");
    Serial.print("  Charge Cap    : "); Serial.print(chargeCap); Serial.println(" Ah");
    return;
  }

  // For other groups: show raw (if you want)
  // Keep minimal printing to avoid flooding
  Serial.print("Group 0x"); Serial.print(header, HEX); Serial.print(" Raw Data: ");
  for (int i = 0; i < len; ++i) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX); Serial.print(' ');
  }
  Serial.println();
}

void printSummary() {
  Serial.println("---- Pack summary (periodic) ----");
  Serial.print("Detected cells: "); Serial.println(numCellsDetected);
  // print first 24 cells to keep output short
  int toShow = min(numCellsDetected, 24);
  for (int i = 0; i < toShow; ++i) {
    Serial.print("C"); Serial.print(i+1); Serial.print(": ");
    if (cellVoltages[i] > 0.0f) {
      Serial.print(cellVoltages[i], 3); Serial.print(" V");
    } else {
      Serial.print("----");
    }
    Serial.print("  ");
    if ((i+1) % 4 == 0) Serial.println();
  }
  Serial.println();
  // show some module temps (first 8)
  Serial.print("Module temps: ");
  for (int i = 0; i < 8; ++i) {
    if (moduleTemps[i] > -100.0f) {
      Serial.print(moduleTemps[i], 1); Serial.print("C ");
    } else {
      Serial.print("-- ");
    }
  }
  Serial.println("\n-------------------------------");
}


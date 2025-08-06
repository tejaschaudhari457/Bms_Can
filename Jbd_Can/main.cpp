#include <Arduino.h>
#include <CAN.h>

// === CAN RX and TX Pins ===
#ifndef CAN_RX_GPIO
#define CAN_RX_GPIO 16
#endif

#ifndef CAN_TX_GPIO
#define CAN_TX_GPIO 17
#endif

// === Global Variables ===
volatile bool dataReceived = false;
uint8_t receivedData[8];

// === CRC-16 as per JBD Spec (Modbus-style, 0xA001, Big-Endian) ===
uint16_t crc16(const uint8_t* data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// === Function Prototypes ===
void onReceive(int packetSize);
void sendRequest_0x100();
void parseAndPrint_0x100();

void setup() {
  Serial.begin(115200);
  delay(1000);

  CAN.setPins(CAN_RX_GPIO, CAN_TX_GPIO);
  if (!CAN.begin(500E3)) {
    Serial.println("CAN Init failed");
    while (1);
  }

  Serial.println("CAN Initialized (500k)");
  CAN.onReceive(onReceive);
  sendRequest_0x100();
}

void loop() {
  static unsigned long lastRequest = 0;

  if (dataReceived) {
    dataReceived = false;

    // Debug dump outside interrupt
    Serial.print("Received [0x100]: ");
    for (int i = 0; i < 8; i++) {
      Serial.print("0x"); Serial.print(receivedData[i], HEX); Serial.print(" ");
    }
    Serial.println();

    uint16_t calcCRC = crc16(receivedData, 6);
    uint16_t expectedCRC = (receivedData[6] << 8) | receivedData[7]; // ✅ FIXED: big-endian

    Serial.print("Calculated CRC : 0x"); Serial.println(calcCRC, HEX);
    Serial.print("Expected CRC   : 0x"); Serial.println(expectedCRC, HEX);

    if (calcCRC == expectedCRC) {
      parseAndPrint_0x100();
    } else {
      Serial.println("❌ CRC mismatch!");
    }

    lastRequest = millis();
  }

  if (millis() - lastRequest > 2000) {
    Serial.println("🔁 Retrying request...");
    sendRequest_0x100();
    lastRequest = millis();
  }
}

// === ISR: Copy Data Only ===
void onReceive(int packetSize) {
  if (CAN.packetId() == 0x100 && packetSize == 8) {
    for (int i = 0; i < 8; i++) {
      receivedData[i] = CAN.read();
    }
    dataReceived = true;
  } else {
    while (CAN.available()) CAN.read(); // flush invalid packet
  }
}

// === Send CAN Request Frame for 0x100 ===
void sendRequest_0x100() {
  CAN.beginPacket(0x100, 0); // Send remote frame (RTR) to request data
  CAN.endPacket();
  Serial.println("📤 Sent request for 0x100");
}

// === Parse and Print Response ===
void parseAndPrint_0x100() {
  uint16_t rawVoltage = (receivedData[0] << 8) | receivedData[1];
  int16_t rawCurrent  = (receivedData[2] << 8) | receivedData[3];
  uint16_t rawCapacity = (receivedData[4] << 8) | receivedData[5];

  float voltage = rawVoltage / 100.0;      // 10mV unit → volts
  float current = rawCurrent / 100.0;      // 10mA unit → amps
  float capacity = rawCapacity / 100.0;    // 10mAh unit → Ah

  Serial.println("✔ Valid Data from 0x100:");
  Serial.print("  🔋 Total Voltage : "); Serial.print(voltage); Serial.println(" V");
  Serial.print("  ⚡ Current       : "); Serial.print(current); Serial.println(" A");
  Serial.print("  🧪 Capacity      : "); Serial.print(capacity); Serial.println(" Ah");
}

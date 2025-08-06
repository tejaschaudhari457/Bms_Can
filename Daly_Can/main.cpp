
/*INIT FILE

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

#include <Arduino.h>
#include <CAN.h>

// === CAN RX and TX Pins (adjust as per your wiring) ===
#ifndef CAN_RX_GPIO
#define CAN_RX_GPIO 16
#endif

#ifndef CAN_TX_GPIO
#define CAN_TX_GPIO 17
#endif

// === Global Variables ===
volatile bool dataReceived = false;
uint8_t receivedData[8];

// === Function Prototypes ===
void onReceive(int packetSize);
void sendRequest_0x90();
void parseAndPrint_0x90();

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(1000); // Optional: allow Serial monitor to catch up

  // Initialize CAN
  CAN.setPins(CAN_RX_GPIO, CAN_TX_GPIO);
  if (!CAN.begin(250E3)) {
    Serial.println("Failed to initialize CAN");
    while (1); // Halt if CAN init fails
  }
  Serial.println("CAN Initialized");

  // Set interrupt handler
  CAN.onReceive(onReceive);

  // Prepare first request
  sendRequest_0x90();
}

// === Loop ===
void loop() {
  static unsigned long lastRequestTime = 0;

  if (dataReceived) {
    dataReceived = false;
    parseAndPrint_0x90();

    // Wait 2 seconds before next request
    lastRequestTime = millis();
  }

  // Resend request every 2 seconds if no response
  if (millis() - lastRequestTime > 2000) {
    Serial.println("No response. Retrying...");
    sendRequest_0x90();
    lastRequestTime = millis();
  }
}

// === Interrupt: onReceive ===
void onReceive(int packetSize) {
  uint32_t packetId = CAN.packetId();

  // Check if response matches 0x90 response format: 0x18 90 40 01
  if (packetId == 0x18904001 && packetSize == 8) {
    for (int i = 0; i < 8; i++) {
      receivedData[i] = CAN.read();
    }
    dataReceived = true;
  } else {
    // Clear buffer if it's not the expected ID
    while (CAN.available()) CAN.read();
  }
}

// === Function: Send 0x90 Request ===
void sendRequest_0x90() {
  const uint32_t requestId = 0x18900140; // Extended ID for 0x90

  CAN.beginExtendedPacket(requestId, 8);
  for (int i = 0; i < 8; i++) CAN.write(0x00); // Reserved/empty payload
  CAN.endPacket();

  Serial.println("Sent request for 0x90");
}

// === Function: Parse and Print 0x90 Data ===
void parseAndPrint_0x90() {
  float totalVoltage      = ((receivedData[0] << 8) | receivedData[1]) / 10.0;
  float acquisitionVoltage = ((receivedData[2] << 8) | receivedData[3]) / 10.0;
  float totalCurrent      = (((receivedData[4] << 8) | receivedData[5]) - 30000) / 10.0;
  float SOC               = ((receivedData[6] << 8) | receivedData[7]) / 10.0;

  Serial.println("Received 0x90 Frame:");
  Serial.print("  Total Voltage      : "); Serial.print(totalVoltage); Serial.println(" V");
  Serial.print("  Acquisition Voltage: "); Serial.print(acquisitionVoltage); Serial.println(" V");
  Serial.print("  Total Current      : "); Serial.print(totalCurrent); Serial.println(" A");
  Serial.print("  State of Charge    : "); Serial.print(SOC); Serial.println(" %");
}


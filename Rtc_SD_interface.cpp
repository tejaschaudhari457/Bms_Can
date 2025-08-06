#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SdFat.h>

// ----- RTC and SD Definitions -----
RTC_DS3231 rtc;
#define SD_CS_PIN 5
SdFat sd;
File logFile;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize I2C for RTC (ESP32 specific pins)
  Wire.begin(21, 22);

  if (!rtc.begin()) {
    Serial.println("❌ RTC not found. Check wiring.");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("⚠️ RTC lost power. Setting default time.");
    rtc.adjust(DateTime(2025, 7, 21, 6, 55, 0)); // Optional default time
  }

  // Initialize SPI for SD card
  SPI.begin(18, 19, 23, SD_CS_PIN);
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(10))) {
    Serial.println("❌ SD card init failed.");
    while (1);
  }

  Serial.println("✅ SD card initialized.");
}

void loop() {
  DateTime now = rtc.now();

  // Create filename in ISO 8601 (safe) format: YYYY-MM-DDTHHMMSS+0000.csv
  char filename[32];
  snprintf(filename, sizeof(filename),
           "%04d-%02d-%02dT%02d%02d%02d+0000.csv",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  // Prepare ISO timestamp inside file with colon-separated format
  char timestamp[32];
  snprintf(timestamp, sizeof(timestamp),
           "%04d-%02d-%02dT%02d:%02d:%02d+00:00",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  // Dummy data generation
  float temperature = random(200, 300) / 10.0; // e.g., 20.0 - 30.0
  float voltage = random(320, 340) / 100.0;    // e.g., 3.20 - 3.40
  const char* status = "OK";

  // Create and write file
  logFile = sd.open(filename, FILE_WRITE);
  if (logFile) {
    logFile.println("Timestamp,Temperature,Voltage,Status");
    logFile.printf("%s,%.1f,%.2f,%s\n", timestamp, temperature, voltage, status);
    logFile.close();
    Serial.printf("✅ File created: %s\n", filename);
  } else {
    Serial.println("❌ File creation failed.");
  }

  delay(5000); // Wait 5 seconds before next file
}

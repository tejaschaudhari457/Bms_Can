/*[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_port = COM10
monitor_port = COM10

lib_deps = 
    https://github.com/greiman/SdFat.git
    bblanchon/ArduinoJson@^6.21.2*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <SdFat.h>
#include <set>
#include <vector>
#include <algorithm>

// ─────────────── WiFi Configuration ───────────────
const char *ssid = "Airtel_Lithium_India";
const char *password = "Lithium@677781";

// ─────────────── Server & Customer Configuration ───────────────
const char *serverHost = "192.168.1.27";
const int serverPort = 8000;
const char *customerId = "CODM9921"; // <-- Change this ID when assigning to a new customer
String getAPI = String("http://") + serverHost + ":" + serverPort + "/api/files/last-updated/" + customerId;
String uploadAPI = "/api/files/upload/" + String(customerId);

// ─────────────── SD Card Setup ───────────────
const uint8_t chipSelect = 5;
SdFat SD;
SdFile file;

// ─────────────── Convert dd-mm-yyyy to sortable integer ───────────────
int dateToInt(const String &date) {
  return date.substring(6, 10).toInt() * 10000 + date.substring(3, 5).toInt() * 100 + date.substring(0, 2).toInt();
}

// ─────────────── Upload multiple files in one POST ───────────────
bool uploadFilesBatch(const std::vector<String> &fileNames) {
  String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
  String endRequest = "\r\n--" + boundary + "--\r\n";

  int contentLength = 0;
  std::vector<String> fileHeaders;
  std::vector<uint32_t> fileSizes;

  // Prepare headers and lengths
  for (const String &fileName : fileNames) {
    if (!file.open(fileName.c_str(), O_READ)) {
      Serial.println("❌ Failed to open: " + fileName);
      return false;
    }

    uint32_t size = file.fileSize();
    file.close();

    String header = "--" + boundary + "\r\n";
    header += "Content-Disposition: form-data; name=\"files\"; filename=\"" + fileName + "\"\r\n";
    header += "Content-Type: text/csv\r\n\r\n";

    fileHeaders.push_back(header);
    fileSizes.push_back(size);
    contentLength += header.length() + size + 2;
  }

  contentLength += endRequest.length();

  // Connect
  WiFiClient client;
  if (!client.connect(serverHost, serverPort)) {
    Serial.println("❌ Server connection failed");
    return false;
  }

  // Send HTTP POST headers
  client.println("POST " + uploadAPI + " HTTP/1.1");
  client.println("Host: " + String(serverHost));
  client.println("Content-Type: multipart/form-data; boundary=" + boundary);
  client.println("Content-Length: " + String(contentLength));
  client.println("Connection: close");
  client.println();

  // Send files
  for (size_t i = 0; i < fileNames.size(); ++i) {
    client.print(fileHeaders[i]);

    if (file.open(fileNames[i].c_str(), O_READ)) {
      uint8_t buf[64];
      int n;
      while ((n = file.read(buf, sizeof(buf))) > 0) {
        client.write(buf, n);
      }
      file.close();
    }

    client.print("\r\n");
  }

  client.print(endRequest);

  // Read response
  bool success = false;
  while (client.connected()) {
    while (client.available()) {
      String line = client.readStringUntil('\n');
      if (line.indexOf("200") != -1) success = true;
      Serial.println(line);
    }
  }

  client.stop();
  return success;
}

// ─────────────── Upload with retry up to 3 times ───────────────
bool uploadWithRetry(const std::vector<String> &batch) {
  for (int attempt = 1; attempt <= 3; ++attempt) {
    Serial.printf("\U0001F4E4 Uploading batch (attempt %d)...\n", attempt);
    if (uploadFilesBatch(batch)) {
      Serial.println("✅ Batch upload successful");
      return true;
    }
    Serial.println("⚠️ Upload failed, retrying...");
    delay(2000);
  }

  Serial.println("❌ All upload attempts failed.");
  return false;
}

// ─────────────── Main Setup ───────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Connect WiFi
  Serial.println("🔌 Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi Connected");

  // Initialize SD Card
  Serial.print("📂 Initializing SD card...");
  if (!SD.begin(chipSelect, SD_SCK_MHZ(10))) {
    Serial.println("❌ SD card init failed!");
    return;
  }
  Serial.println("✅ SD card initialized");

  // ───── GET Last Uploaded Date from Server ─────
  HTTPClient http;
  http.begin(getAPI);
  int httpResponseCode = http.GET();

  String lastUploadedDate = "00-00-0000";
  bool useFallbackMode = false;

  if (httpResponseCode == 200) {
    String response = http.getString();
    Serial.println("✅ GET API response:\n" + response);

    int idx = response.indexOf("\"lastUpdatedDate\"");
    if (idx != -1) {
      int start = response.indexOf("\"", idx + 18);
      int end = response.indexOf("\"", start + 1);
      lastUploadedDate = response.substring(start + 1, end);
      if (lastUploadedDate.length() == 9) {
        lastUploadedDate = "0" + lastUploadedDate;
      }
    }
    Serial.println("📅 Last uploaded date: " + lastUploadedDate);

  } else if (httpResponseCode == 404) {
    String response = http.getString();
    if (response.indexOf("Files do not exist for this deviceId") != -1) {
      useFallbackMode = true;
      Serial.println("⚠️ No uploaded files on server. Entering fallback mode...");
    } else {
      Serial.println("❌ GET failed with unexpected 404.");
      return;
    }
  } else {
    Serial.printf("❌ GET request failed with code: %d\n", httpResponseCode);
    return;
  }

  http.end();

  // ───── Scan SD Card for .csv Files ─────
  SdFile dir;
  if (!dir.open("/")) {
    Serial.println("❌ Failed to open SD root directory");
    return;
  }

  SdFile entry;
  std::vector<String> matchedFiles;
  std::vector<String> allDates;

  while (entry.openNext(&dir, O_READ)) {
    char nameBuf[64];
    entry.getName(nameBuf, sizeof(nameBuf));
    String fileName = String(nameBuf);

    if (fileName.endsWith(".csv") && fileName.length() >= 14) {
      String fileDate = fileName.substring(0, 10);  // Format: dd-mm-yyyy.csv
      allDates.push_back(fileDate + "|" + fileName);
    }

    entry.close();
  }

  dir.close();

  // Sort files descending by date
  std::sort(allDates.begin(), allDates.end(), [](const String &a, const String &b) {
    return dateToInt(a.substring(0, 10)) > dateToInt(b.substring(0, 10));
  });

  if (useFallbackMode) {
    // ───── Send last 15 files only ─────
    std::set<String> addedDates;
    for (const String &entry : allDates) {
      String date = entry.substring(0, 10);
      String file = entry.substring(11);
      if (addedDates.size() < 15) {
        matchedFiles.push_back(file);
        addedDates.insert(date);
      }
    }
  } else {
    // ───── Upload all files newer than last uploaded date ─────
    for (const String &entry : allDates) {
      String date = entry.substring(0, 10);
      String file = entry.substring(11);
      if (dateToInt(date) > dateToInt(lastUploadedDate)) {
        matchedFiles.push_back(file);
      } else {
        Serial.println("⏩ Skipping old file: " + file);
      }
    }
  }

  if (matchedFiles.empty()) {
    Serial.println("📁 No files to upload.");
    return;
  }

  // ───── Upload Files in Batches of 10 ─────
  for (size_t i = 0; i < matchedFiles.size(); i += 10) {
    std::vector<String> batch;
    for (size_t j = i; j < i + 10 && j < matchedFiles.size(); ++j) {
      batch.push_back(matchedFiles[j]);
    }

    if (!uploadWithRetry(batch)) {
      Serial.println("🚫 Aborting further uploads due to failure.");
      break;
    }

    delay(2000);  // optional delay between batches
  }
}

void loop() {
  // Nothing here
}

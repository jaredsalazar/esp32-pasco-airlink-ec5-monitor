#include <Arduino.h>
#include <NimBLEDevice.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace {

constexpr char kTargetDeviceName[] = "AirLink";
constexpr uint8_t kSensorServiceId = 1;
constexpr uint8_t kDeviceServiceId = 0;
constexpr uint8_t kSendCmdCharId = 2;
constexpr uint8_t kRecvCmdCharId = 3;
constexpr uint8_t kSendAckCharId = 5;

constexpr uint8_t kCmdReadOneSample = 0x05;
constexpr uint8_t kCmdGetSensorId = 0x08;
constexpr uint8_t kRspResult = 0xC0;
constexpr uint8_t kEvtSensorId = 0x82;

constexpr uint32_t kScanTimeSeconds = 5;
constexpr uint32_t kPollIntervalMs = 1000;
constexpr uint32_t kResponseTimeoutMs = 1500;
constexpr uint16_t kSensorIdPs2163 = 257;
constexpr uint16_t kSensorIdWirelessSoilMoisture = 2065;
constexpr bool kVerbosePackets = false;

struct SoilReading {
  bool valid = false;
  float relativeWetness = NAN;
  float pottingSoil = NAN;
  float mineralSoil = NAN;
  float rockwool = NAN;
  float waterPotential = NAN;
};

NimBLEClient *g_client = nullptr;
NimBLERemoteCharacteristic *g_deviceCommandChar = nullptr;
NimBLERemoteCharacteristic *g_deviceNotifyChar = nullptr;
NimBLERemoteCharacteristic *g_sensorCommandChar = nullptr;
NimBLERemoteCharacteristic *g_sensorNotifyChar = nullptr;
NimBLERemoteCharacteristic *g_sensorAckChar = nullptr;

volatile bool g_sampleReady = false;
volatile bool g_sensorIdReady = false;
volatile bool g_bridgeReady = false;
uint16_t g_lastSensorId = 0;
std::string g_lastSamplePayload;
void requestSensorId();
void requestBridgeActivation();
void handleNotify(const char *source, const std::string &value);
void printHex(const std::string &data);

String uuidFor(uint8_t serviceId, uint8_t characteristicId) {
  char buffer[37];
  snprintf(buffer, sizeof(buffer), "4a5c000%u-000%u-0000-0000-5c1e741f1c00",
           serviceId, characteristicId);
  return String(buffer);
}

float clampPercent(float value) {
  return std::max(0.0f, std::min(100.0f, value));
}

float computeRelativeWetness(const SoilReading &reading) {
  const float pottingNormalized = reading.pottingSoil / 50.0f;
  const float mineralNormalized = reading.mineralSoil / 50.0f;
  const float rockwoolNormalized = reading.rockwool / 75.0f;
  const float averageNormalized =
      (pottingNormalized + mineralNormalized + rockwoolNormalized) / 3.0f;
  return clampPercent(averageNormalized * 100.0f);
}

void logWrite(const char *target, const std::string &value) {
  if (!kVerbosePackets) {
    return;
  }
  Serial.print("-> [");
  Serial.print(target);
  Serial.print("] ");
  printHex(value);
  Serial.println();
}

float decodePasco1616(const std::string &payload, size_t offset) {
  const uint32_t raw32 =
      static_cast<uint32_t>(static_cast<uint8_t>(payload[offset])) |
      (static_cast<uint32_t>(static_cast<uint8_t>(payload[offset + 1])) << 8) |
      (static_cast<uint32_t>(static_cast<uint8_t>(payload[offset + 2])) << 16) |
      (static_cast<uint32_t>(static_cast<uint8_t>(payload[offset + 3])) << 24);
  return static_cast<float>(static_cast<int32_t>(raw32)) / 65536.0f;
}

SoilReading decodeSoilReading(const std::string &payload) {
  SoilReading reading;

  if (g_lastSensorId == kSensorIdPs2163) {
    if (payload.size() < 16) {
      return reading;
    }

    reading.valid = true;
    reading.pottingSoil = clampPercent(decodePasco1616(payload, 0));
    reading.mineralSoil = clampPercent(decodePasco1616(payload, 4));
    reading.rockwool = clampPercent(decodePasco1616(payload, 8));
    reading.waterPotential = decodePasco1616(payload, 12);
    if (reading.mineralSoil < 0.01f && reading.rockwool < 0.01f &&
        reading.pottingSoil > 2.0f) {
      reading.valid = false;
    } else {
      reading.relativeWetness = computeRelativeWetness(reading);
    }
    return reading;
  }
  return reading;
}

void printHex(const std::string &data) {
  for (size_t i = 0; i < data.size(); ++i) {
    if (i) {
      Serial.print(' ');
    }
    uint8_t value = static_cast<uint8_t>(data[i]);
    if (value < 0x10) {
      Serial.print('0');
    }
    Serial.print(value, HEX);
  }
}

void handleNotify(const char *source, const std::string &value) {
  if (value.empty()) {
    return;
  }

  if (kVerbosePackets) {
    Serial.print("[");
    Serial.print(source);
    Serial.print("] ");
    printHex(value);
    Serial.println();
  }

  const uint8_t header = static_cast<uint8_t>(value[0]);

  if (header == kEvtSensorId && value.size() >= 2) {
    g_lastSensorId = static_cast<uint8_t>(value[1]);
    if (value.size() >= 3) {
      g_lastSensorId |=
          static_cast<uint16_t>(static_cast<uint8_t>(value[2])) << 8;
    }
    g_sensorIdReady = true;
    Serial.print("Attached sensor event ID from ");
    Serial.print(source);
    Serial.print(": 0x");
    if (g_lastSensorId < 0x10) {
      Serial.print('0');
    }
    Serial.println(g_lastSensorId, HEX);
    if (g_lastSensorId == kSensorIdPs2163) {
      Serial.println("Detected PASCO Soil Moisture Sensor PS-2163 profile.");
    } else if (g_lastSensorId == kSensorIdWirelessSoilMoisture) {
      Serial.println("Detected PASCO Wireless Soil Moisture profile.");
    }
    return;
  }

  if (header != kRspResult || value.size() < 3) {
    return;
  }

  const uint8_t status = static_cast<uint8_t>(value[1]);
  const uint8_t command = static_cast<uint8_t>(value[2]);

  if (status != 0x00) {
    return;
  }

  if (command == kCmdReadOneSample && value.size() >= 5) {
    if (g_lastSensorId == kSensorIdPs2163 && strcmp(source, "device") != 0) {
      return;
    }
    g_lastSamplePayload.assign(value.begin() + 3, value.end());
    g_sampleReady = true;
    return;
  }

  if (strcmp(source, "sensor") == 0 && command == 0x01) {
    g_bridgeReady = true;
    Serial.println("AirLink bridge became active, requesting attached sensor ID...");
    requestSensorId();
    return;
  }

}

void deviceNotifyCallback(NimBLERemoteCharacteristic *, uint8_t *data, size_t length, bool) {
  handleNotify("device", std::string(reinterpret_cast<char *>(data), length));
}

void sensorNotifyCallback(NimBLERemoteCharacteristic *, uint8_t *data, size_t length, bool) {
  handleNotify("sensor", std::string(reinterpret_cast<char *>(data), length));
}

bool connectToAirLink() {
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(15);

  Serial.println("Scanning for AirLink...");
  NimBLEScanResults results = scan->start(kScanTimeSeconds, false);
  NimBLEAdvertisedDevice *advertisedDevice = nullptr;
  for (int i = 0; i < results.getCount(); ++i) {
    NimBLEAdvertisedDevice device = results.getDevice(i);
    if (!device.haveName()) {
      continue;
    }

    const std::string name = device.getName();
    if (name.find(kTargetDeviceName) == std::string::npos) {
      continue;
    }

    Serial.print("Found target: ");
    Serial.println(name.c_str());
    advertisedDevice = new NimBLEAdvertisedDevice(device);
    break;
  }

  if (advertisedDevice == nullptr) {
    Serial.println("No AirLink found.");
    return false;
  }

  g_client = NimBLEDevice::createClient();
  if (!g_client->connect(advertisedDevice)) {
    Serial.println("Failed to connect to AirLink.");
    delete advertisedDevice;
    return false;
  }
  delete advertisedDevice;

  const String serviceUuid = uuidFor(kSensorServiceId, 0);
  const String deviceServiceUuid = uuidFor(kDeviceServiceId, 0);
  NimBLERemoteService *deviceService =
      g_client->getService(deviceServiceUuid.c_str());
  NimBLERemoteService *sensorService =
      g_client->getService(serviceUuid.c_str());
  if (deviceService == nullptr || sensorService == nullptr) {
    Serial.println("PASCO device or sensor service not found.");
    return false;
  }

  g_deviceCommandChar =
      deviceService->getCharacteristic(uuidFor(kDeviceServiceId, kSendCmdCharId).c_str());
  g_deviceNotifyChar =
      deviceService->getCharacteristic(uuidFor(kDeviceServiceId, kRecvCmdCharId).c_str());
  g_sensorCommandChar =
      sensorService->getCharacteristic(uuidFor(kSensorServiceId, kSendCmdCharId).c_str());
  g_sensorNotifyChar =
      sensorService->getCharacteristic(uuidFor(kSensorServiceId, kRecvCmdCharId).c_str());
  g_sensorAckChar =
      sensorService->getCharacteristic(uuidFor(kSensorServiceId, kSendAckCharId).c_str());

  if (g_deviceCommandChar == nullptr || g_deviceNotifyChar == nullptr ||
      g_sensorCommandChar == nullptr || g_sensorNotifyChar == nullptr) {
    Serial.println("PASCO command characteristics are missing.");
    return false;
  }

  if (!g_deviceNotifyChar->canNotify() || !g_sensorNotifyChar->canNotify()) {
    Serial.println("PASCO notify characteristic does not support notifications.");
    return false;
  }

  if (!g_deviceNotifyChar->subscribe(true, deviceNotifyCallback)) {
    Serial.println("Failed to subscribe to PASCO device notifications.");
    return false;
  }

  if (!g_sensorNotifyChar->subscribe(true, sensorNotifyCallback)) {
    Serial.println("Failed to subscribe to PASCO sensor notifications.");
    return false;
  }

  Serial.println("Connected to AirLink.");
  if (g_sensorAckChar == nullptr) {
    Serial.println("Ack characteristic not found. One-shot reads may still work.");
  }
  return true;
}

bool ensureAirLinkConnected() {
  if (g_client != nullptr && g_client->isConnected()) {
    return true;
  }

  while (!connectToAirLink()) {
    Serial.println("Retrying AirLink scan in 2 seconds...");
    delay(2000);
  }

  requestBridgeActivation();
  return true;
}

bool waitForFlag(volatile bool &flag, uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (!flag && (millis() - start) < timeoutMs) {
    delay(10);
  }
  return flag;
}

void requestSensorId() {
  g_sensorIdReady = false;
  const std::string command(1, static_cast<char>(kCmdGetSensorId));
  logWrite("sensor", command);
  if (!g_sensorCommandChar->writeValue(command, false)) {
    Serial.println("Failed to request attached sensor ID.");
    return;
  }

  if (!waitForFlag(g_sensorIdReady, kResponseTimeoutMs)) {
    Serial.println("No attached sensor ID response received.");
  }
}

void requestBridgeActivation() {
  g_bridgeReady = false;
  const std::string command(1, static_cast<char>(kCmdGetSensorId));
  logWrite("device", command);
  if (!g_deviceCommandChar->writeValue(command, false)) {
    Serial.println("Failed to activate AirLink bridge.");
    return;
  }

  if (!waitForFlag(g_bridgeReady, kResponseTimeoutMs)) {
    Serial.println("No AirLink bridge response received.");
  }
}

bool requestSoilSample(SoilReading &reading) {
  const uint32_t deadline = millis() + kResponseTimeoutMs;

  while (millis() < deadline) {
    g_sampleReady = false;
    g_lastSamplePayload.clear();

    uint8_t payloadBytes = 2;
    if (g_lastSensorId == kSensorIdPs2163) {
      payloadBytes = 16;
    }

    const std::string command = {
        static_cast<char>(kCmdReadOneSample),
        static_cast<char>(payloadBytes),
    };

    logWrite("sensor-read", command);
    if (!g_sensorCommandChar->writeValue(command, false)) {
      Serial.println("Failed to request soil sample.");
      return false;
    }

    const uint32_t remainingMs = deadline - millis();
    if (!waitForFlag(g_sampleReady, remainingMs)) {
      break;
    }

    reading = decodeSoilReading(g_lastSamplePayload);
    if (reading.valid) {
      return true;
    }
  }

  Serial.println("Timed out waiting for a valid soil sample.");
  return false;
}
void printReading(const SoilReading &reading) {
  if (!reading.valid) {
    Serial.println("Sample payload was too short to decode.");
    return;
  }

  if (g_lastSensorId == kSensorIdPs2163) {
    Serial.print("Relative Wetness: ");
    Serial.print(reading.relativeWetness, 1);
    Serial.print("%  ");
    Serial.print("VWC Potting Soil: ");
    Serial.print(reading.pottingSoil, 1);
    Serial.print("%  Mineral Soil: ");
    Serial.print(reading.mineralSoil, 1);
    Serial.print("%  Rockwool: ");
    Serial.print(reading.rockwool, 1);
    Serial.print("%");
    if (fabsf(reading.waterPotential) > 0.001f) {
      Serial.print("  Water Potential: ");
      Serial.print(reading.waterPotential, 2);
    }
    Serial.println();
    return;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("PASCO AirLink EC-5 monitor starting...");
  Serial.println("Waiting to identify the attached PASCO soil moisture profile.");

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
}

void loop() {
  static uint32_t lastPollMs = 0;
  static uint32_t lastWaitingLogMs = 0;

  if (!ensureAirLinkConnected()) {
    delay(1000);
    return;
  }

  if (!g_sensorIdReady) {
    if (millis() - lastWaitingLogMs > 3000) {
      lastWaitingLogMs = millis();
      Serial.println("Waiting for AirLink sensor handshake...");
    }
    delay(50);
    return;
  }

  if (millis() - lastPollMs < kPollIntervalMs) {
    delay(10);
    return;
  }
  lastPollMs = millis();

  SoilReading reading;
  if (requestSoilSample(reading)) {
    printReading(reading);
  }
}

#ifndef TINY_GSM_MODEM_SIM7600
#define TINY_GSM_MODEM_SIM7600
#endif

#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 1024
#endif

#include <Arduino.h>
#include <PubSubClient.h>
#include <TinyGsmClient.h>

#include <cstdio>
#include <cstring>

#ifndef MQTT_HOST
#define MQTT_HOST "tcp.ap-southeast-1.clawcloudrun.com"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 39024
#endif

#ifndef MQTT_TOPIC
#define MQTT_TOPIC "esp32/pasco"
#endif

#ifndef MODEM_BAUD
#define MODEM_BAUD 115200
#endif

#ifndef MODEM_RX_PIN
#define MODEM_RX_PIN 27
#endif

#ifndef MODEM_TX_PIN
#define MODEM_TX_PIN 26
#endif

#ifndef MODEM_DTR_PIN
#define MODEM_DTR_PIN 25
#endif

#ifndef MODEM_PWRKEY_PIN
#define MODEM_PWRKEY_PIN 4
#endif

#ifndef MODEM_RESET_PIN
#define MODEM_RESET_PIN 5
#endif

#ifndef MODEM_POWER_ON_PIN
#define MODEM_POWER_ON_PIN 12
#endif

#ifndef CELLULAR_APN
#define CELLULAR_APN "internet"
#endif

#ifndef CELLULAR_USER
#define CELLULAR_USER ""
#endif

#ifndef CELLULAR_PASS
#define CELLULAR_PASS ""
#endif

#ifndef MODEM_RESET_LEVEL
#define MODEM_RESET_LEVEL HIGH
#endif

#ifndef MODEM_POWERON_PULSE_WIDTH_MS
#define MODEM_POWERON_PULSE_WIDTH_MS 100
#endif

#ifndef MODEM_START_WAIT_MS
#define MODEM_START_WAIT_MS 3000
#endif

namespace {

constexpr uint32_t kCellularRetryIntervalMs = 5000;
constexpr uint32_t kMqttRetryIntervalMs = 5000;
constexpr uint32_t kNetworkTimeoutMs = 60000;
constexpr uint32_t kPublishIntervalMs = 10000;

HardwareSerial SerialAT(1);
TinyGsm g_modem(SerialAT);
TinyGsmClient g_cellularClient(g_modem);
PubSubClient g_mqttClient(g_cellularClient);

bool g_modemReady = false;
bool g_serialAtReady = false;
uint32_t g_lastCellularAttemptMs = 0;
uint32_t g_lastMqttAttemptMs = 0;
uint32_t g_lastPublishMs = 0;
uint32_t g_bootCount = 0;

void configureBoardPins() {
  pinMode(MODEM_POWER_ON_PIN, OUTPUT);
  digitalWrite(MODEM_POWER_ON_PIN, HIGH);

  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);

  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
  delay(2600);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
}

void powerOnModem() {
  digitalWrite(MODEM_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
  delay(MODEM_POWERON_PULSE_WIDTH_MS);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);
}

String mqttClientId() {
  const uint64_t chipId = ESP.getEfuseMac();
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "esp32-hello-%08lx",
           static_cast<unsigned long>(chipId & 0xffffffff));
  return String(buffer);
}

bool ensureCellularConnected() {
  if (millis() - g_lastCellularAttemptMs < kCellularRetryIntervalMs) {
    return g_modemReady && g_modem.isNetworkConnected() && g_modem.isGprsConnected();
  }
  g_lastCellularAttemptMs = millis();

  if (g_modemReady && g_modem.isNetworkConnected() && g_modem.isGprsConnected()) {
    return true;
  }

  if (!g_modemReady) {
    Serial.println("Powering A7670...");
    configureBoardPins();
    powerOnModem();

    if (!g_serialAtReady) {
      SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
      g_serialAtReady = true;
    }

    Serial.println("Start modem...");
    delay(MODEM_START_WAIT_MS);

    if (!g_modem.init()) {
      Serial.println("modem.init() failed");
      return false;
    }

    g_modemReady = true;
    Serial.print("Modem name: ");
    Serial.println(g_modem.getModemName());
    Serial.print("Modem info: ");
    Serial.println(g_modem.getModemInfo());
  }

  Serial.println("Waiting for network...");
  if (!g_modem.waitForNetwork(kNetworkTimeoutMs)) {
    Serial.println("Network registration timed out.");
    return false;
  }

  Serial.print("Opening data session with APN: ");
  Serial.println(CELLULAR_APN);
  if (!g_modem.gprsConnect(CELLULAR_APN, CELLULAR_USER, CELLULAR_PASS)) {
    Serial.println("Failed to open cellular data session.");
    return false;
  }

  Serial.print("Cellular IP: ");
  Serial.println(g_modem.localIP());
  return true;
}

bool ensureMqttConnected() {
  if (!ensureCellularConnected()) {
    return false;
  }

  if (g_mqttClient.connected()) {
    return true;
  }

  if (millis() - g_lastMqttAttemptMs < kMqttRetryIntervalMs) {
    return false;
  }
  g_lastMqttAttemptMs = millis();

  g_mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  Serial.print("Connecting MQTT to ");
  Serial.print(MQTT_HOST);
  Serial.print(':');
  Serial.println(MQTT_PORT);

  if (g_mqttClient.connect(mqttClientId().c_str())) {
    Serial.println("MQTT connected.");
    return true;
  }

  Serial.print("MQTT connect failed, state=");
  Serial.println(g_mqttClient.state());
  return false;
}

void publishHello() {
  char payload[192];
  snprintf(payload, sizeof(payload),
           "{\"message\":\"hello\",\"bootCount\":%lu,\"millis\":%lu,"
           "\"chip\":\"%s\",\"modem\":\"A7670\"}",
           static_cast<unsigned long>(g_bootCount),
           static_cast<unsigned long>(millis()),
           mqttClientId().c_str());

  if (g_mqttClient.publish(MQTT_TOPIC, payload, true)) {
    Serial.print("Published hello to ");
    Serial.println(MQTT_TOPIC);
  } else {
    Serial.println("Failed to publish hello.");
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  ++g_bootCount;
  Serial.println();
  Serial.println("A7670 hello MQTT test starting...");
  g_mqttClient.setBufferSize(256);
}

void loop() {
  g_mqttClient.loop();

  if (!ensureMqttConnected()) {
    delay(100);
    return;
  }

  if (g_lastPublishMs == 0 || millis() - g_lastPublishMs >= kPublishIntervalMs) {
    g_lastPublishMs = millis();
    publishHello();
  }

  delay(50);
}

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <microDS18B20.h>

#define DS_PIN 8

// Pin configuration
const uint8_t PROXY_PIN = 21;
const uint8_t LOCK1_PIN = 7;
const uint8_t LOCK2_PIN = 27;

// Timers
const unsigned long TEMP_INTERVAL_MS = 600000; // 10 minutes
const unsigned long PROX_INTERVAL_MS = 500;    // poll proximity frequently

// Network / MQTT
byte mac[] = {0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 61);
const char* MQTT_SERVER = "192.168.1.3";
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "arduino-1";
const char* MQTT_SUB_TOPIC = "/bob/status";
const char* MQTT_PUB_TOPIC = "BOB";

// Temp sensor addresses
uint8_t s1_addr[] = {0x28, 0x79, 0x6A, 0x49, 0xF6, 0xDE, 0x3C, 0x0A};
uint8_t s2_addr[] = {0x28, 0xC9, 0x4C, 0xDB, 0x32, 0x20, 0x01, 0x44};

MicroDS18B20<DS_PIN, s1_addr> sensor1;
MicroDS18B20<DS_PIN, s2_addr> sensor2;

int proxyState = LOW;
unsigned long lastTempMs = 0;
unsigned long lastProxMs = 0;

EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

StaticJsonDocument<258> pubJson;
StaticJsonDocument<500> subJson;

void applyLockState(bool open) {
  digitalWrite(LOCK1_PIN, open ? HIGH : LOW);
  digitalWrite(LOCK2_PIN, open ? HIGH : LOW);
}

void publishJson(const char* topic) {
  char buffer[128];
  serializeJson(pubJson, buffer, sizeof(buffer));
  if (!mqttClient.publish(topic, buffer)) {
    Serial.println("MQTT publish failed");
  }
}

template <typename Sensor>
void publishTemperature(Sensor& sensor, const char* label) {
  Serial.print(label);
  if (sensor.readTemp()) {
    float temp = sensor.getTemp();
    Serial.println(temp);
    pubJson.clear();
    pubJson.add(temp);
    publishJson(MQTT_PUB_TOPIC);
  } else {
    Serial.println("error");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Enter callback");
  deserializeJson(subJson, payload, length);

  String extinsion_id = subJson["extinsion"];
  String door_state = String(subJson["status"][0]["door"]);
  String lock_state = String(subJson["status"][0]["lock"]);

  int p = (char)payload[0] - '0';
  if (p == 0) {
    applyLockState(false);
    Serial.println("Lock is closed");
  } else if (p == 1) {
    applyLockState(true);
    Serial.println("Lock is open");
  }

  Serial.print("extension number: ");
  Serial.println(extinsion_id);
  Serial.print("door_state: ");
  Serial.println(door_state);
  Serial.print("lock_state: ");
  Serial.println(lock_state);
}

void setup() {
  Ethernet.init(17);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  Ethernet.begin(mac, ip);
  delay(1500);

  pinMode(LOCK1_PIN, OUTPUT);
  pinMode(LOCK2_PIN, OUTPUT);
  pinMode(PROXY_PIN, INPUT);

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(callback);

  if (mqttClient.connect(MQTT_CLIENT_ID)) {
    Serial.println("Connected");
    bool r = mqttClient.subscribe(MQTT_SUB_TOPIC);
    Serial.print("subscribe: ");
    Serial.println(r);
  } else {
    Serial.println("Connection failed");
  }
}

void handleTemperatures(unsigned long now) {
  if (now - lastTempMs < TEMP_INTERVAL_MS) return;
  lastTempMs = now;

  sensor1.requestTemp();
  sensor2.requestTemp();
  delay(1000); // wait for conversions

  publishTemperature(sensor1, "t1: ");
  publishTemperature(sensor2, "t2: ");
}

void handleProximity(unsigned long now) {
  if (now - lastProxMs < PROX_INTERVAL_MS) return;
  lastProxMs = now;

  int val = digitalRead(PROXY_PIN);
  if (val != proxyState) {
    proxyState = val;
  }

  pubJson.clear();
  JsonArray array = pubJson.to<JsonArray>();

  if (proxyState == LOW) {
    array.add("Not detected");
    applyLockState(true);
    array.add("Unlock");
  } else {
    array.add("Detected");
    applyLockState(false);
    array.add("Lock");
  }

  publishJson(MQTT_PUB_TOPIC);
  Serial.println("Proximity update published");
}

void loop() {
  unsigned long now = millis();

  handleTemperatures(now);
  handleProximity(now);

  mqttClient.loop();
  delay(1000);
}

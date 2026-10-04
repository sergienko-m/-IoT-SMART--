/*
 * Практична робота 2.1 — Розумна теплиця
 * ESP32 + MQTT + DHT22 + capacitive soil + BH1750 + MQ-135 + DS18B20
 * Актуатори: pump, fan PWM, grow light PWM, window servo, heater.
 * Перед прошивкою замініть Wi-Fi/MQTT реквізити.
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>
#include <EEPROM.h>

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define MQTT_HOST "192.168.1.10"
#define MQTT_PORT 1883
#define MQTT_USER "greenhouse"
#define MQTT_PASSWORD "change_me"
#define DEVICE_ID "esp32-greenhouse-01"
#define ROOT "greenhouse/zone1"

// Sensor pins
const uint8_t DHT_PIN = 4, DHT_TYPE = DHT22;
const uint8_t SOIL_PIN = 34, MQ135_PIN = 35, PRESSURE_PIN = 33, PIR_PIN = 32;
const uint8_t ONE_WIRE_PIN = 5;
// Actuator pins
const uint8_t PUMP_PIN = 25, FAN_PIN = 26, LIGHT_PIN = 27, SERVO_PIN = 13;
const uint8_t HEATER_PIN = 14;
const uint8_t FAN_CH = 0, LIGHT_CH = 1;

DHT dht(DHT_PIN, DHT_TYPE);
BH1750 lightMeter;
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature soilTemp(&oneWire);
Servo windowServo;
WiFiClient wifi;
PubSubClient mqtt(wifi);

struct Calibration { int soilDry; int soilWet; float mqBaseline; } cal;
const int EEPROM_SIZE = 64;
unsigned long lastPublish = 0, lastWifiAttempt = 0, lastMqttAttempt = 0;
const unsigned long PUBLISH_MS = 10000;

float validFloat(float v, float lo, float hi, float fallback) {
  return isfinite(v) && v >= lo && v <= hi ? v : fallback;
}
void publishText(const String &topic, const String &value, bool retained = true) {
  if (mqtt.connected()) mqtt.publish(topic.c_str(), value.c_str(), retained);
}
void publishFloat(const String &topic, float value, uint8_t decimals = 2) {
  if (isfinite(value)) publishText(topic, String(value, decimals));
}
void setActuator(const char *topic, bool state) { publishText(String(ROOT) + "/actuators/" + topic + "/state", state ? "ON" : "OFF"); }

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiAttempt < 5000) return;
  lastWifiAttempt = millis();
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String t(topic), msg; for (unsigned int i=0; i<len; i++) msg += (char)payload[i];
  msg.toUpperCase();
  if (t.endsWith("/pump/command")) digitalWrite(PUMP_PIN, msg == "ON" ? HIGH : LOW);
  else if (t.endsWith("/heater/command")) digitalWrite(HEATER_PIN, msg == "ON" ? HIGH : LOW);
  else if (t.endsWith("/fan/command")) ledcWrite(FAN_CH, constrain(msg.toInt(),0,100) * 255 / 100);
  else if (t.endsWith("/light/command")) ledcWrite(LIGHT_CH, constrain(msg.toInt(),0,100) * 255 / 100);
  else if (t.endsWith("/window/command")) windowServo.write(constrain(msg.toInt(),0,100) * 1.8 + 0);
  publishText(String(ROOT) + "/diagnostics/last_command", t + "=" + msg, false);
}
void ensureMQTT() {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  if (millis() - lastMqttAttempt < 5000) return;
  lastMqttAttempt = millis();
  String client = String(DEVICE_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (mqtt.connect(client.c_str(), MQTT_USER, MQTT_PASSWORD,
      (String(ROOT)+"/diagnostics/availability").c_str(), 1, true, "offline")) {
    mqtt.publish((String(ROOT)+"/diagnostics/availability").c_str(), "online", true);
    mqtt.subscribe((String(ROOT)+"/actuators/+/command").c_str());
  }
}
void calibrateSensors() {
  EEPROM.begin(EEPROM_SIZE); EEPROM.get(0, cal);
  if (cal.soilDry < 100 || cal.soilDry > 4090 || cal.soilWet < 100 || cal.soilWet > 4090 || cal.soilDry == cal.soilWet) {
    long sum = 0; for (int i=0;i<30;i++) { sum += analogRead(SOIL_PIN); delay(50); }
    cal.soilDry = sum / 30; cal.soilWet = max(300, cal.soilDry - 900); cal.mqBaseline = analogRead(MQ135_PIN);
    EEPROM.put(0, cal); EEPROM.commit();
  }
  publishText(String(ROOT)+"/diagnostics/calibration", "soilDry="+String(cal.soilDry)+",soilWet="+String(cal.soilWet));
}
float soilPercent(int raw) { return constrain(100.0 * (cal.soilDry - raw) / (float)(cal.soilDry - cal.soilWet), 0, 100); }
void publishSensors() {
  float airT = validFloat(dht.readTemperature(), -20, 70, NAN);
  float airH = validFloat(dht.readHumidity(), 0, 100, NAN);
  float lux = validFloat(lightMeter.readLightLevel(), 0, 100000, NAN);
  soilTemp.requestTemperatures(); float groundT = validFloat(soilTemp.getTempCByIndex(0), -30, 80, NAN);
  int soilRaw = analogRead(SOIL_PIN), mqRaw = analogRead(MQ135_PIN);
  float soil = soilPercent(soilRaw), co2Index = max(0.0f, (mqRaw - cal.mqBaseline) * 1.2f + 400.0f);
  publishFloat(String(ROOT)+"/sensors/air/temperature", airT);
  publishFloat(String(ROOT)+"/sensors/air/humidity", airH);
  publishFloat(String(ROOT)+"/sensors/soil/moisture", soil, 1);
  publishFloat(String(ROOT)+"/sensors/soil/temperature", groundT);
  publishFloat(String(ROOT)+"/sensors/light/lux", lux, 1);
  publishFloat(String(ROOT)+"/sensors/air/co2_index", co2Index, 0);
  publishText(String(ROOT)+"/sensors/soil/raw", String(soilRaw));
  publishText(String(ROOT)+"/sensors/air_quality/raw", String(mqRaw));
  publishText(String(ROOT)+"/sensors/presence", digitalRead(PIR_PIN) ? "ON" : "OFF");
  publishText(String(ROOT)+"/sensors/pressure", digitalRead(PRESSURE_PIN) ? "OK" : "LOW");
  bool valid = isfinite(airT) && isfinite(airH) && isfinite(groundT) && isfinite(lux) && soil >= 0 && soil <= 100;
  publishText(String(ROOT)+"/diagnostics/data_valid", valid ? "ON" : "OFF");
  publishText(String(ROOT)+"/diagnostics/uptime_s", String(millis()/1000));
}
void setup() {
  Serial.begin(115200); pinMode(PUMP_PIN,OUTPUT); pinMode(HEATER_PIN,OUTPUT); pinMode(PRESSURE_PIN,INPUT_PULLUP); pinMode(PIR_PIN,INPUT);
  dht.begin(); Wire.begin(21,22); lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE); soilTemp.begin();
  ledcSetup(FAN_CH, 25000, 8); ledcAttachPin(FAN_PIN, FAN_CH); ledcSetup(LIGHT_CH, 1000, 8); ledcAttachPin(LIGHT_PIN, LIGHT_CH);
  windowServo.attach(SERVO_PIN); windowServo.write(0); digitalWrite(PUMP_PIN,LOW); digitalWrite(HEATER_PIN,LOW);
  mqtt.setServer(MQTT_HOST, MQTT_PORT); mqtt.setCallback(mqttCallback); calibrateSensors();
}
void loop() {
  connectWiFi(); ensureMQTT(); mqtt.loop();
  if (millis() - lastPublish >= PUBLISH_MS) { lastPublish = millis(); publishSensors(); }
  delay(10);
}

#include <Arduino.h>
#include <AsyncMqttClient.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <Ticker.h>
#include <Wire.h>

#include "creds.h"

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

#define DHTTYPE DHT22 // DHT 22

#define MQTT_HOST IPAddress(192, 168, 1, 2)
#define MQTT_PORT 1883

#define TEMPERATURE_TOPIC "house/hall/temperature"
#define HUMIDITY_TOPIC "house/hall/humidity"

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

char humidityBuff[5];
char temperatureBuff[5];

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP &event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected &event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while
                               // reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe("test/lol", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  mqttClient.publish("test/lol", 0, true, "test 1");
  Serial.println("Publishing at QoS 0");
  uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
  uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  Serial.print("Publishing at QoS 2, packetId: ");
  Serial.println(packetIdPub2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload,
                   AsyncMqttClientMessageProperties properties, size_t len,
                   size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);
// DHT Sensor
uint8_t DHTPin = D3;

// Initialize DHT sensor.
DHT dht(DHTPin, DHTTYPE);
// output control pins
#define LCD_CONTRAST_PIN 6
#define LCD_BACKLIGHT_PIN 10

void setup() {
  lcd.begin(lcdColumns, lcdRows); // initialize LCD
  lcd.init();

  pinMode(DHTPin, INPUT);

  dht.begin();

  lcd.backlight();

  Wire.begin();
  Serial.begin(115200);

  Serial.println();
  Serial.println();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}

void loop() {
  float temperature = dht.readTemperature();          // Gets the values of the temperature
  float humidity = dht.readHumidity();                // Gets the values of the humidity
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT");
  } else {
    Serial.print("Temperatura: ");
    Serial.println(temperature);
    Serial.print("Wilgotność: ");
    Serial.println(humidity);

    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print(String("TEMP: ") + String(temperature));
    lcd.print((char)223); // Degree symbol
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print(String("HUM: ") + String(humidity) + String("%"));

    sprintf(humidityBuff, "%d", (int) humidity);
    sprintf(temperatureBuff, "%d", (int) temperature);
    mqttClient.publish(HUMIDITY_TOPIC, 0, false, humidityBuff);
    mqttClient.publish(TEMPERATURE_TOPIC, 0, false, temperatureBuff);
  }
  delay(2000);
  lcd.clear();
}

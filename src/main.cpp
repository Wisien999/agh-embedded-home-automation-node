#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

#include "creds.h"


WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

#define MQTT_HOST IPAddress(192, 168, 1, 2)
#define MQTT_PORT 1883
Ticker mqttReconnectTimer;
AsyncMqttClient mqttClient;

// set the LCD number of columns and rows
#define lcdColumns 16
#define lcdRows 2
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  
char lcd_message[lcdRows * lcdColumns + 1 + 1];

#define DHT_PIN D4
Ticker dhtTicker;
DHT22 dht_sensor;
volatile float humidity = 0;
volatile float temperature = 0;
volatile uint8_t dht_error = 0;
volatile int8_t dht_result = 0;

Ticker light_ticker;
volatile uint16_t light_value = 0;

#define PIR_PIN D5
uint8_t prev_movement = LOW; // LOW is no movement

#define DOOR_PIN D7
uint8_t prev_door = LOW; // LOW is door closed


void read_DHT() {
  dht_sensor.read();
}

// this callback will be called from an interrupt
// it should be short and carry the ICACHE_RAM_ATTR attribute
void ICACHE_RAM_ATTR handleData(float h, float t) {
  humidity = h;
  temperature = t;
  dht_result = 1;
}

// this callback will be called from an interrupt
// it should be short and carry the ICACHE_RAM_ATTR attribute
void ICACHE_RAM_ATTR handleError(uint8_t e) {
  Serial.print("DHT error occurred. Error code: ");
  Serial.println(e);
  Serial.print("Error: ");
  Serial.println(dht_sensor.getError());
  dht_error = e;
  dht_result = -1;
}


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  uint16_t packetIdPub1 = mqttClient.publish("hall/connected", 1, false, "online");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void read_light() {
  light_value = analogRead(A0);
  if (mqttClient.connected()) {
    char val_str[5];
    sprintf(val_str, "%d", light_value);
    mqttClient.publish("hall/light/level", 0, false, val_str);
  }
}


void setup(){
  lcd.begin(lcdColumns, lcdRows);// initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print("Waiting");
  lcd.setCursor(0, 1);
  lcd.print("for read");

  pinMode(DHT_PIN, INPUT);
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT);
  
  dht_sensor.setPin(DHT_PIN);
  dht_sensor.onData(handleData);
  dht_sensor.onError(handleError);
  dhtTicker.attach(5, read_DHT);

  light_ticker.attach(120, read_light);
  
  Wire.begin();
  Serial.begin(115200);

  Serial.println();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}


void loop(){
  uint8_t current_door_status = digitalRead(DOOR_PIN);
  if (current_door_status != prev_door) {
    prev_door = current_door_status;
    mqttClient.publish("hall/exit-door", 0, false, current_door_status == HIGH ? "opened" : "closed");
  }
  uint8_t current_motion = digitalRead(PIR_PIN);
  if (current_motion != prev_movement) {
    prev_movement = current_motion;
    mqttClient.publish("hall/movement", 0, false, current_motion == HIGH ? "motion" : "still");
  }

  if (dht_result > 0) {
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);

    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print(String("TEMP: ") + String(temperature) );
    lcd.print((char)223); //Degree symbol
    lcd.print("C");
    lcd.setCursor(0,1);
    lcd.print( String("HUM: ") +  String(humidity) + String("%"));
  }

  delay(2000);
}

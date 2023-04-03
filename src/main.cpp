#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <Wire.h>

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

#define DHTTYPE DHT22   // DHT 22


LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  
// DHT Sensor
uint8_t DHTPin = D3; 
               
// Initialize DHT sensor.
DHT dht(DHTPin, DHTTYPE);                
 // output control pins
#define LCD_CONTRAST_PIN 6
#define LCD_BACKLIGHT_PIN 10

void setup(){
  lcd.begin( lcdColumns, lcdRows);// initialize LCD
  lcd.init();

  pinMode(DHTPin, INPUT);
  
  dht.begin();      
  
  lcd.backlight();
 
  Wire.begin();
  Serial.begin(115200);
 
}


void loop(){
  
  float Temperature = dht.readTemperature(); // Gets the values of the temperature
  float Humidity = dht.readHumidity(); // Gets the values of the humidity 
  if (isnan(Temperature) || isnan(Humidity)){
    Serial.println("Failed to read from DHT");
  }
  else {
    Serial.print("Temperatura: ");
    Serial.println(Temperature);
    Serial.print("Wilgotność: ");
    Serial.println(Humidity);

    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print(String("TEMP: ") + String(Temperature) );
    lcd.print((char)223 ); //Degree symbol
    lcd.print("C");
    lcd.setCursor(0,1);
    lcd.print( String("HUM: ") +  String(Humidity) + String("%"));

  }
  delay(2000);
  lcd.clear(); 

}

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>  // Use Adafruit_BMP280.h if using BMP280
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>

// Define the type of DHT sensor
#define DHTTYPE DHT22   // DHT 22 (AM2302)

// Define the pin for the DHT sensor
#define DHTPIN 2

// Replace with your network credentials
const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

// Create instances of the sensors
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);  // Use Adafruit_BMP280 bmp if using BMP280
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Setup Wi-Fi client
WiFiClient client;

void setup() {
  Serial.begin(115200);
  dht.begin();
  if(!bmp.begin()) {
    Serial.print("Could not find a valid BMP085 sensor, check wiring!");
    while (1);
  }
  lcd.begin();
  lcd.backlight();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  // Read data from DHT sensor
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Read data from BMP sensor
  sensors_event_t event;
  bmp.getEvent(&event);

  // Display data on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(t);
  lcd.print(" H:");
  lcd.print(h);

  if (event.pressure) {
    lcd.setCursor(0, 1);
    lcd.print("P:");
    lcd.print(event.pressure);
  }

  // Send data to server or cloud (optional)
  // You can use an API or your own server to send data

  delay(2000); // Wait for 2 seconds before updating
}

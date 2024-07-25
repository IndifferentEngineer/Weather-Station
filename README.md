---

# Weather Station

A simple IoT-based weather station that monitors and displays real-time weather data such as temperature, humidity, and atmospheric pressure using an Arduino. The data is displayed on an LCD screen and can be sent to a web dashboard or smartphone app via Wi-Fi.

## Components Needed

- Arduino Uno (or any compatible board)
- DHT11/DHT22 Sensor (for temperature and humidity)
- BMP180/BMP280 Sensor (for atmospheric pressure)
- Wi-Fi Module (ESP8266 or ESP32)
- LCD Display (16x2 or 20x4 with I2C module)
- Jumper wires
- Breadboard
- Power supply (battery or USB)

## Wiring Diagram

### DHT11/DHT22 Sensor
- VCC to 5V on Arduino
- GND to GND on Arduino
- Data to Digital Pin 2 on Arduino

### BMP180/BMP280 Sensor
- VCC to 3.3V on Arduino
- GND to GND on Arduino
- SCL to A5 on Arduino (or SCL on some boards)
- SDA to A4 on Arduino (or SDA on some boards)

### Wi-Fi Module (ESP8266)
- VCC to 3.3V on Arduino
- GND to GND on Arduino
- TX to RX on Arduino (through a voltage divider if necessary)
- RX to TX on Arduino

### LCD Display (with I2C module)
- VCC to 5V on Arduino
- GND to GND on Arduino
- SCL to A5 on Arduino (or SCL on some boards)
- SDA to A4 on Arduino (or SDA on some boards)

## Installation

### Step 1: Install Libraries

1. **Open the Arduino IDE.**
2. **Go to the Library Manager:**
   - Click on `Sketch` > `Include Library` > `Manage Libraries...`
3. **Search for and install the following libraries:**
   - **Adafruit Unified Sensor**
   - **Adafruit BMP085 Unified** (or `Adafruit BMP280` if you're using the BMP280 sensor)
   - **DHT sensor library**
   - **LiquidCrystal I2C**

### Step 2: Upload the Code

1. **Connect your Arduino board to your computer.**
2. **Select the correct board and port:**
   - Go to `Tools` > `Board` > `Arduino Uno`.
   - Go to `Tools` > `Port` and select the port your Arduino is connected to.
3. **Upload the code:**
   - Click the `Upload` button in the Arduino IDE.

### Code

```cpp
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
```

## Troubleshooting

- Ensure all components are connected properly and securely.
- Make sure the necessary libraries are installed in the Arduino IDE.
- Double-check your Wi-Fi credentials and ensure the Wi-Fi module is functioning.

## Acknowledgments

- [Adafruit Industries](https://www.adafruit.com/) for providing excellent sensors and libraries.
- The Arduino community for continuous support and contributions.

---


[Link to Repository](https://github.com/IndifferentEngineer/Weather-Station)

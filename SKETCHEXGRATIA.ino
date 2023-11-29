#include <Wire.h>
#include <WiFi.h>
#include "ThingSpeak.h"
#include "Adafruit_PM25AQI.h"
#include <SoftwareSerial.h>
#include <MQUnifiedsensor.h>
#include <Arduino.h>

// WiFi credentials
#define SECRET_SSID "Aida's Galaxy Tab A8"    
#define SECRET_PASS "0134886291"
#define SECRET_CH_ID 2343382
#define SECRET_WRITE_APIKEY "71JLCMIYYIE9KU75"

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int keyIndex = 0;
WiFiClient client;

unsigned long myChannelNumber = SECRET_CH_ID;
const char *myWriteAPIKey = SECRET_WRITE_APIKEY;

// Define PM25 sensor pins and instances
SoftwareSerial pmSerial(16, 17); // RX, TX pins
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

const int mq135Pin = 34; // Pin connected to the analog output of MQ135 sensor
const int soundSensorPin = 35; // Sound sensor pin

// SHT30 I2C address
#define SHT30_I2C_ADDR 0x44

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pmSerial.begin(9600);

  if (!aqi.begin_UART(&pmSerial)) {
    Serial.println("Could not find PM 2.5 sensor!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("PM25 sensor found!");

  pinMode(mq135Pin, INPUT);
  pinMode(soundSensorPin, INPUT);

  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);
}

float get_CO_ppm(float voltage) {
  // Function to calculate CO concentration in ppm
  // You need to calibrate this function based on your sensor's characteristics
  // Placeholder code is provided for demonstration purposes
  
  // Example calibration values (adjust according to your sensor's characteristics)
  float slope = -0.5; // Adjust the slope for higher values
  float intercept = 3.0; // Adjust the intercept for higher values
  
  // Use the adjusted slope and intercept to calculate CO concentration in ppm
  float CO_ppm = slope * voltage + intercept;
  
  return CO_ppm;
}

void loop() {
  // Read PM2.5 sensor data
  PM25_AQI_Data data;

  if (!aqi.read(&data)) {
    Serial.println("Could not read from AQI sensor");
    delay(500);
    return;
  }

  int pm25std = data.pm25_env;
  int pm100std = data.pm100_env;

  // Read sound levels from sound sensor
  int maxDbValue = -1;
  for (int i = 0; i < 10; i++) {
    int rawValue = analogRead(soundSensorPin);
    int dBValue = map(rawValue, 1950, 2220, 50, 70);
    dBValue = constrain(dBValue, 40, 88);
    if (dBValue > maxDbValue) {
      maxDbValue = dBValue;
    }
    delay(100);
  }

  // Start SHT30 measurement
  Wire.beginTransmission(SHT30_I2C_ADDR);
  Wire.write(0x2C);
  Wire.write(0x06);
  Wire.endTransmission();
  delay(1000); // Wait for measurement to complete (adjust the delay based on your needs)

  // Read CO2 levels from MQ135
  int sensorValue = analogRead(mq135Pin);
  float voltage = sensorValue * (3.3 / 4095.0);
  float CO_ppm = get_CO_ppm(voltage); // Calculate CO concentration in ppm

  // Read temperature and humidity from SHT30
  Wire.requestFrom(SHT30_I2C_ADDR, 6);
  while (Wire.available() < 6);
  uint16_t rawTemperature = Wire.read() << 8 | Wire.read();
  Wire.read(); // Skip CRC
  uint16_t rawHumidity = Wire.read() << 8 | Wire.read();
  Wire.read(); // Skip CRC

  float temperature = -45 + 175 * (rawTemperature / 65535.0);
  float humidity = 100 * (rawHumidity / 65535.0);

  // Print sensor readings to Serial Monitor
  Serial.print("PM2.5: "); Serial.println(pm25std);
  Serial.print("PM10: "); Serial.println(pm100std);
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
  Serial.print("CO Concentration: "); Serial.print(CO_ppm); Serial.println(" ppm");
  Serial.print("Max Sound dB: "); Serial.println(maxDbValue);

  // Send sensor readings to ThingSpeak
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
    Serial.println("Connected");
  }

  ThingSpeak.setField(1, pm25std);
  ThingSpeak.setField(2, pm100std);
  ThingSpeak.setField(3, temperature);
  ThingSpeak.setField(4, humidity);
  ThingSpeak.setField(5, CO_ppm);
  ThingSpeak.setField(6, maxDbValue);

  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200) {
    Serial.println("Channel update successful.");
  } else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
  delay(15000); // Wait for next update
}

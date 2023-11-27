#include <Wire.h>
#include <WiFi.h>
#include "ThingSpeak.h"
#include "Adafruit_PM25AQI.h"
#include <SoftwareSerial.h>
#include <MQUnifiedsensor.h>
#include <Arduino.h>

// WiFi credentials
#define SECRET_SSID "4G-MIFI-DEDB"    
#define SECRET_PASS "12345678"
#define SECRET_CH_ID 2343382
#define SECRET_WRITE_APIKEY "71JLCMIYYIE9KU75"

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int keyIndex = 0;
WiFiClient client;

unsigned long myChannelNumber = SECRET_CH_ID;
const char *myWriteAPIKey = SECRET_WRITE_APIKEY;

// Define PM25 sensor pins and instances
SoftwareSerial pmSerial(16, 17);
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

// Define MQ135 sensor pin
#define placa "ESP32"
#define Voltage_Resolution 3.3
#define pin 34 // Analog input 0 of your arduino
#define type "MQ-135" // MQ135
#define ADC_Bit_Resolution 12 // For ESP32
#define RatioMQ135CleanAir 3.6 // RS / R0 = 3.6 ppm

MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

// Sound sensor pin
const int soundSensorPin = 35;

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
  Serial.println("PM25 found!");

  pinMode(pin, INPUT);
  pinMode(soundSensorPin, INPUT);

  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);

  // Initialize MQ135 sensor
  MQ135.setRegressionMethod(1); // _PPM = a*ratio^b
  MQ135.init();

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0)) { Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while (1); }
  if (calcR0 == 0) { Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while (1); }

  Serial.println("** Values from MQ-135 ****");
  Serial.println("|    CO   |   CO2  |");
}

void loop() {
  // Read PM2.5 sensor data
  PM25_AQI_Data data;

  if (!aqi.read(&data)) {
    Serial.println("Could not read from AQI");
    delay(500);
    return;
  }

  int pm25std = data.pm25_env;
  int pm100std = data.pm100_env;

  // Read sound levels from max9814 sensor
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
  MQ135.update();

  MQ135.setA(605.18); MQ135.setB(-3.937);
  float CO = MQ135.readSensor();

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
  Serial.print(F("Temperature: ")); Serial.print(temperature); Serial.println(" °C");
  Serial.print(F("Humidity: ")); Serial.print(humidity); Serial.println(" %");
  Serial.print("CO Concentration: "); Serial.print(CO); Serial.println(" ppm");
  Serial.print("Max Sound dB: "); Serial.println(maxDbValue);
  Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
  Serial.print("Particles > 10 um / 0.1L air:"); Serial.println(data.particles_100um);

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
  ThingSpeak.setField(5, CO);
  ThingSpeak.setField(6, maxDbValue);
  ThingSpeak.setField(7, data.particles_25um);      
  ThingSpeak.setField(8, data.particles_100um);

  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200) {
    Serial.println("Channel update successful.");
  } else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
  delay(15000); // Wait for next update
}
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <WiFiEsp.h>
#include <SPI.h>

// DS18B20 Setup (Temperature)
#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// MPU6050 Setup (Vibration)
Adafruit_MPU6050 mpu;

// Wi-Fi Credentials (Make sure to replace with your own credentials)
const char* ssid = "YourSSID";
const char* password = "YourPassword";

// Server Details
const char* server = "yourserver.com"; // Replace with your server's IP or domain
const int port = 80;

// Voltage Divider Calibration Values
const float voltageDividerRatio = 11.0; // Example value for a voltage divider of 3.3MΩ and 10kΩ

// Arduino Analog Pins for Voltage Measurement (3 phases)
const int phaseA_pin = A0;
const int phaseB_pin = A1;
const int phaseC_pin = A2;

// Calibration for Current Sensor (e.g., CSNB121)
const float currentOffset = 512.0; // Adjust this based on your sensor's idle value
const float currentScale = 5.0;    // Adjust this based on your sensor's scale (e.g., for 5A range)

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // Use Serial1 for WiFi communication (Arduino Mega)

  // Initialize DS18B20
  sensors.begin();

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  // Initialize Wi-Fi
  WiFi.init(&Serial1);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    delay(5000);
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to Wi-Fi!");
  } else {
    Serial.println("Wi-Fi connection failed.");
  }
}

float readVoltage(int pin) {
  int sensorValue = analogRead(pin); // Read the analog pin value (0-1023)
  float voltage = sensorValue * (5.0 / 1023.0); // Convert to voltage (assuming 5V reference)
  voltage = voltage * voltageDividerRatio; // Apply the voltage divider ratio
  return voltage;
}

float readCurrent(int pin) {
  int sensorValue = analogRead(pin); // Read the current sensor value
  float current = (sensorValue - currentOffset) * (5.0 / 1023.0); // Convert to current (V to A)
  current = current * currentScale; // Apply scale factor for the current sensor
  return current;
}

void loop() {
  // Read DS18B20 Temperature
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // Read MPU6050 Vibration Data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  // Read Voltage (3 phases)
  float voltageA = readVoltage(phaseA_pin);
  float voltageB = readVoltage(phaseB_pin);
  float voltageC = readVoltage(phaseC_pin);

  // Read Current (Assuming current sensor on pin A3)
  float currentA = readCurrent(A3);
  float currentB = readCurrent(A4); // Assuming current sensor on pin A4
  float currentC = readCurrent(A5); // Assuming current sensor on pin A5

  // Calculate Power (Simple P = V * I, adjust formula as needed for your application)
  float powerA = voltageA * currentA;
  float powerB = voltageB * currentB;
  float powerC = voltageC * currentC;

  // Send Data to Server
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    if (client.connect(server, port)) {
      String postData = "temperature=" + String(tempC) +
                        "&voltageA=" + String(voltageA) +
                        "&voltageB=" + String(voltageB) +
                        "&voltageC=" + String(voltageC) +
                        "&currentA=" + String(currentA) +
                        "&currentB=" + String(currentB) +
                        "&currentC=" + String(currentC) +
                        "&powerA=" + String(powerA) +
                        "&powerB=" + String(powerB) +
                        "&powerC=" + String(powerC) +
                        "&accelX=" + String(accelX) +
                        "&accelY=" + String(accelY) +
                        "&accelZ=" + String(accelZ);
      
      client.println("POST /data HTTP/1.1");
      client.println("Host: " + String(server));
      client.println("Connection: close");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.println("Content-Length: " + String(postData.length()));
      client.println();
      client.println(postData);
      client.stop();
      Serial.println("Data sent to server!");
    }
  }
  
  delay(15000); // Send data every 15 seconds
}

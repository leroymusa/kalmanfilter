#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <RH_RF95.h>
#include <SPI.h>
//#include "imu_math/quaternion.hpp"
#include "kalman_filter.h"
#include "ArduinoJson.h"

#define RFM95_CS 1
#define RFM95_RST 34
#define RFM95_INT 8

Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BMP280 bmp;


#define RF95_FREQ 915.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Define your JSON buffer size here
const size_t bufferSize = JSON_OBJECT_SIZE(6);

Kalman kalman(&bmp, &bno);

void setup() {
  Serial.begin(9600);

  if (!bno.begin())
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  // Initialize radio
  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Set frequency
  if (!rf95.setFrequency(915.0)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.println("Frequency set to 915.0MHz");

  kalman.begin();
}

void loop() {
  // Update Kalman filter
  kalman.update();

  // Prepare data for JSON
  StaticJsonDocument<200> jsonDoc; // Create JSON document object


  jsonDoc["QuatW"] = kalman.getQuaternionW(); // Add temperature data to JSON object
  //jsonDoc["pressure"] = pressure; // Add pressure data to JSON object
  
  // Serialize JSON to string
  String jsonString;
  serializeJson(jsonDoc, jsonString); // Serialize JSON to string

  // Send JSON data over LoRa
  rf95.send((uint8_t*)jsonString.c_str(), jsonString.length()); // Send the JSON string over LoRa
  rf95.waitPacketSent(); // Wait for the packet to be sent

  /*
  jsonBuffer["altitude"] = kalman.getAltitude();
  jsonBuffer["velocity"] = kalman.getVelocity();
  jsonBuffer["quaternion_w"] = kalman.getQuaternionW();
  jsonBuffer["quaternion_x"] = kalman.getQuaternionX();
  jsonBuffer["quaternion_y"] = kalman.getQuaternionY();
  jsonBuffer["quaternion_z"] = kalman.getQuaternionZ();

  // Serialize JSON to a string
  String jsonString;
  serializeJson(jsonBuffer, jsonString);

  // Convert string to char array
  char jsonCharArray[jsonString.length() + 1];
  jsonString.toCharArray(jsonCharArray, jsonString.length() + 1);

  // Send JSON data via LoRa
  rf95.send((uint8_t *)jsonCharArray, strlen(jsonCharArray));
  rf95.waitPacketSent();
  */
  // Print JSON string
  Serial.println("JSON data sent:");
  Serial.println(jsonString);
}

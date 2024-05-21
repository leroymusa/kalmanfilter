#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include "KalmanFilter.h"

// Sensor objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(115200);
  
  // Initialize BNO055 IMU
  if (!bno.begin()) {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  // Initialize BMP280
  if (!bmp.begin(0x76)) {
    Serial.print("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  
  // Initialize Kalman filter
  kalmanInit();
}

void loop() {
  // Time step
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Read sensors
  sensors_event_t accelEvent;
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  float accel = accelEvent.acceleration.z;

  float altitude = bmp.readAltitude(1013.25); // Adjust to your local sea level pressure

  // Kalman filter predict and update
  kalmanPredict(dt);
  kalmanUpdate(altitude, accel);

  // Output raw and filtered data
  Serial.print("Raw Altitude: "); Serial.print(altitude);
  Serial.print(" Raw Acceleration: "); Serial.print(accel);
  Serial.print(" Filtered Altitude: "); Serial.print(state[0]);
  Serial.print(" Filtered Velocity: "); Serial.print(state[1]);
  Serial.print(" Filtered Acceleration: "); Serial.println(state[2]);

  delay(100); 

}

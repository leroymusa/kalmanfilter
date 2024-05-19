#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include "kalman_filter.h"

Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BMP280 bmp;

Kalman kalman(&bmp, &bno);

void setup() {
  Serial.begin(9600);

  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  kalman.begin();
}

void loop() {
  kalman.update();
}

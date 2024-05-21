#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <Kalman.h>

// Create sensor objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP280 bmp;

// Kalman filter objects
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;
Kalman kalmanAlt;

// Time variables
unsigned long previousTime = 0;
float dt = 0.0;

void setup() {
  Serial.begin(115200);

  // Initialize BNO055
  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    while (1);
  }
  bno.setExtCrystalUse(true);

  // Initialize BMP280
  if (!bmp.begin()) {
    Serial.print("No BMP280 detected");
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // Initialize Kalman filters
  kalmanX.setAngle(0);
  kalmanY.setAngle(0);
  kalmanZ.setAngle(0);
  kalmanAlt.setAngle(0);

  previousTime = millis();
}

void loop() {
  // Time delta
  unsigned long currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // Read IMU data
  sensors_event_t event;
  bno.getEvent(&event);
  
  float accelX = event.acceleration.x;
  float accelY = event.acceleration.y;
  float accelZ = event.acceleration.z;

  // Assuming the BNO055 provides orientation data directly
  float roll = event.orientation.x;
  float pitch = event.orientation.y;
  float yaw = event.orientation.z;

  // Read altitude from BMP280
  float altitude = bmp.readAltitude(1013.25); // Assuming sea level pressure is 1013.25 hPa

  // Kalman filter prediction and update
  float rollKalman = kalmanX.getAngle(roll, accelX, dt);
  float pitchKalman = kalmanY.getAngle(pitch, accelY, dt);
  float yawKalman = kalmanZ.getAngle(yaw, accelZ, dt);
  float altKalman = kalmanAlt.getAngle(altitude, 0, dt); // 0 as the BMP280 does not provide acceleration data

  // Print filtered values
  Serial.print("Roll: "); Serial.print(rollKalman);
  Serial.print(" Pitch: "); Serial.print(pitchKalman);
  Serial.print(" Yaw: "); Serial.print(yawKalman);
  Serial.print(" Altitude: "); Serial.println(altKalman);

  delay(100);
}

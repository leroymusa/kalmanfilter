#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>

class Kalman {
public:
  Kalman(Adafruit_BMP280 *bmp, Adafruit_BNO055 *bno);

  void init();
  void begin();
  void update();
  
  // Add getter methods for altitude, velocity, and quaternion angles
  double getAltitude() const { return filteredAltitude; }
  double getVelocity() const { return filteredVelocity; }
  double getQuaternionW() const { return current_state[2]; }
  double getQuaternionX() const { return current_state[3]; }
  double getQuaternionY() const { return current_state[4]; }
  double getQuaternionZ() const { return current_state[5]; }
private:
  Adafruit_BMP280 *bmp;
  Adafruit_BNO055 *bno;

  // Kalman filter variables
  double Q[3][3]; // Process noise covariance matrix
  double R[2][2]; // Measurement noise covariance matrix
  double H[2][3]; // Observation matrix
  double current_p_cov[3][3]; // Current state covariance matrix
  double A[3][3]; // State transition matrix

  double predicted_state[3]; // Predicted state
  double predicted_p_cov[3][3]; // Predicted state covariance matrix
  double kalman_gain[3][2]; // Kalman gain matrix
  double adjusted_state[3]; // Adjusted state
  double adjusted_p_cov[3][3]; // Adjusted state covariance matrix

  unsigned long old_time; // Previous time
  unsigned long curr_time; // Current time
  double dt; // Time difference

  // Additional member variables for storing filtered altitude and velocity
  double filteredAltitude;
  double filteredVelocity;

  // State vector: [altitude, velocity, quaternion_w, quaternion_x, quaternion_y, quaternion_z]
  double current_state[6]; // Current state vector

  void predict_state();
  void predict_p_cov();
  void update_gain();
  void adjust_state();
  void adjust_p_cov();
};

#endif

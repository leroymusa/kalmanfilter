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

  double getAltitude() const { return filteredAltitude; }
  double getVelocity() const { return filteredVelocity; }
  double getQuaternionW() const { return current_state[2]; }
  double getQuaternionX() const { return current_state[3]; }
  double getQuaternionY() const { return current_state[4]; }
  double getQuaternionZ() const { return current_state[5]; }

private:
  Adafruit_BMP280 *bmp;
  Adafruit_BNO055 *bno;

  double Q[6][6]; // Process noise covariance matrix
  double R[2][2]; // Measurement noise covariance matrix
  double H[2][6]; // Observation matrix
  double current_p_cov[6][6]; // Current state covariance matrix
  double A[6][6]; // State transition matrix

  double predicted_state[6]; // Predicted state
  double predicted_p_cov[6][6]; // Predicted state covariance matrix
  double kalman_gain[6][2]; // Kalman gain matrix
  double adjusted_state[6]; // Adjusted state
  double adjusted_p_cov[6][6]; // Adjusted state covariance matrix

  unsigned long old_time; // Previous time
  unsigned long curr_time; // Current time
  double dt; // Time difference

  double filteredAltitude;
  double filteredVelocity;

  double current_state[6]; // Current state vector

  void predict_state();
  void predict_p_cov();
  void update_gain();
  void adjust_state();
  void adjust_p_cov();
};

#endif

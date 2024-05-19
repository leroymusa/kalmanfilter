#include "kalman_filter.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>

Kalman::Kalman(Adafruit_BMP280 *bmp, Adafruit_BNO055 *bno) {
    this->bmp = bmp;
    this->bno = bno;
    init();
}

void Kalman::init() {
  // Initialize State Transition Matrix A
  std::fill(&A[0][0], &A[0][0] + sizeof(A) / sizeof(A[0][0]), 0.0);
  A[0][0] = 1.0; A[1][1] = 1.0; 
  A[2][2] = 1.0; A[3][3] = 1.0; A[4][4] = 1.0; A[5][5] = 1.0;

  // Initialize Process Noise Covariance Matrix Q
  std::fill(&Q[0][0], &Q[0][0] + sizeof(Q) / sizeof(Q[0][0]), 0.0);
  for (int i = 0; i < 6; ++i) {
    Q[i][i] = 0.01;
  }

  // Initialize Measurement Noise Covariance Matrix R
  R[0][0] = 3.0; R[0][1] = 0.0;
  R[1][0] = 0.0; R[1][1] = 0.1;

  // Initialize Observation Matrix H
  std::fill(&H[0][0], &H[0][0] + sizeof(H) / sizeof(H[0][0]), 0.0);
  H[0][0] = 1.0; // Altitude measurement
  H[1][5] = 1.0; // Acceleration measurement

  // Initialize current state covariance matrix P
  std::fill(&current_p_cov[0][0], &current_p_cov[0][0] + sizeof(current_p_cov) / sizeof(current_p_cov[0][0]), 0.0);
  for (int i = 0; i < 6; ++i) {
    current_p_cov[i][i] = 1.0;
  }

  filteredAltitude = 0.0;
  filteredVelocity = 0.0;

  // Initialize current state vector
  current_state[0] = bmp->readAltitude(); // Altitude
  current_state[1] = 0.0; // Velocity
  imu::Quaternion q = bno->getQuat();
  current_state[2] = q.w();
  current_state[3] = q.x();
  current_state[4] = q.y();
  current_state[5] = q.z();
}

void Kalman::begin() {
  old_time = millis();
}

void Kalman::update() {
  curr_time = millis();
  dt = ((double)(curr_time - old_time) / 1000.0);
  old_time = curr_time;

  A[0][1] = dt; // Update A matrix with current dt

  predict_state();
  predict_p_cov();
  update_gain();
  adjust_state();
  adjust_p_cov();

  filteredAltitude = adjusted_state[0];
  filteredVelocity = adjusted_state[1];

  // Update current state and covariance for next iteration
  std::copy(std::begin(adjusted_state), std::end(adjusted_state), std::begin(current_state));
  std::copy(&adjusted_p_cov[0][0], &adjusted_p_cov[0][0] + 6 * 6, &current_p_cov[0][0]);

  // Output results
  Serial.print("Filtered Altitude: ");
  Serial.println(filteredAltitude, 4);
  Serial.print("Filtered Velocity: ");
  Serial.println(filteredVelocity, 4);
}

void Kalman::predict_state() {
  for (int i = 0; i < 6; ++i) {
    predicted_state[i] = 0.0;
    for (int j = 0; j < 6; ++j) {
      predicted_state[i] += A[i][j] * current_state[j];
    }
  }
}

void Kalman::predict_p_cov() {
  double AP[6][6] = {0};
  double APA_T[6][6] = {0}; // Transposed APA

  // AP = A * current_p_cov
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      for (int k = 0; k < 6; ++k) {
        AP[i][j] += A[i][k] * current_p_cov[k][j];
      }
    }
  }

  // APA_T = AP * A'
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      for (int k = 0; k < 6; ++k) {
        APA_T[i][j] += AP[i][k] * A[j][k]; // Note that A[j][k] represents A'
      }
      predicted_p_cov[i][j] = APA_T[i][j] + Q[i][j];
    }
  }
}

void Kalman::update_gain() {
  double PHt[6][2] = {0};
  double HPHt[2][2] = {0};
  double HPHtR[2][2] = {0};
  double HPHtR_inv[2][2] = {0};
  double det;

  // PHt = predicted_p_cov * H'
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 2; ++j) {
      PHt[i][j] = 0.0;
      for (int k = 0; k < 6; ++k) {
        PHt[i][j] += predicted_p_cov[i][k] * H[j][k];
      }
    }
  }

  // HPHt = H * PHt
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      HPHt[i][j] = 0.0;
      for (int k = 0; k < 6; ++k) {
        HPHt[i][j] += H[i][k] * PHt[k][j];
      }
      HPHtR[i][j] = HPHt[i][j] + R[i][j];
    }
  }

  // Calculate inverse of HPHtR
  det = HPHtR[0][0] * HPHtR[1][1] - HPHtR[0][1] * HPHtR[1][0];
  HPHtR_inv[0][0] = HPHtR[1][1] / det;
  HPHtR_inv[1][1] = HPHtR[0][0] / det;
  HPHtR_inv[0][1] = -HPHtR[0][1] / det;
  HPHtR_inv[1][0] = -HPHtR[1][0] / det;

  // Calculate Kalman gain: kalman_gain = PHt * HPHtR_inv
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 2; ++j) {
      kalman_gain[i][j] = 0.0;
      for (int k = 0; k < 2; ++k) {
        kalman_gain[i][j] += PHt[i][k] * HPHtR_inv[k][j];
      }
    }
  }
}

void Kalman::adjust_state() {
  double measurement[2] = {bmp->readAltitude(), bno->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL).z()};

  // Adjust state: adjusted_state = predicted_state + kalman_gain * (measurement - H * predicted_state)
  for (int i = 0; i < 2; ++i) {
    double innovation = measurement[i];
    for (int j = 0; j < 6; ++j) {
      innovation -= H[i][j] * predicted_state[j];
    }
    for (int j = 0; j < 6; ++j) {
      adjusted_state[j] += kalman_gain[j][i] * innovation;
    }
  }
}

void Kalman::adjust_p_cov() {
  double KH[6][6] = {0};

  // KH = kalman_gain * H
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      KH[i][j] = 0.0;
      for (int k = 0; k < 2; ++k) {
        KH[i][j] += kalman_gain[i][k] * H[k][j];
      }
    }
  }

  // Update adjusted_p_cov = (I - KH) * predicted_p_cov
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double sum = (i == j) ? 1.0 : 0.0;
      for (int k = 0; k < 6; ++k) {
        sum -= KH[i][k] * predicted_p_cov[k][j];
      }
      adjusted_p_cov[i][j] = sum;
    }
  }
}

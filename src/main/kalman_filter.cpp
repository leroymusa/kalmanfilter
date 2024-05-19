#include "kalman_filter.h"

Kalman::Kalman(Adafruit_BMP280 *bmp, Adafruit_BNO055 *bno) {
    this->bmp = bmp;
    this->bno = bno;
    init();
}

void Kalman::init() {

  A[0][0] = 1.0; A[0][1] = dt; A[0][2] = 0.0;
  A[1][0] = 0.0; A[1][1] = 1.0; A[1][2] = 0.0;
  A[2][0] = 0.0; A[2][1] = 0.0; A[2][2] = 1.0;
  
  // Adjust Q matrix for stationary board (low process noise)
  Q[0][0] = 0.01; Q[0][1] = 0; Q[0][2] = 0;
  Q[1][0] = 0; Q[1][1] = 0.01; Q[1][2] = 0;
  Q[2][0] = 0; Q[2][1] = 0; Q[2][2] = 0.01;

  // Set R matrix for altitude and acceleration measurements
  R[0][0] = 3; R[0][1] = 0;
  R[1][0] = 0; R[1][1] = 0.1;

  // Initialize H matrix for altitude and acceleration measurements
  H[0][0] = 1; H[0][1] = 0; H[0][2] = 0;
  H[1][0] = 0; H[1][1] = 0; H[1][2] = 1;

  // Initialize current state covariance matrix
  current_p_cov[0][0] = 3; current_p_cov[0][1] = 0; current_p_cov[0][2] = 0;
  current_p_cov[1][0] = 0; current_p_cov[1][1] = 2; current_p_cov[1][2] = 0;
  current_p_cov[2][0] = 0; current_p_cov[2][1] = 0; current_p_cov[2][2] = 1;

  filteredAltitude = 0.0;
  filteredVelocity = 0.0;

  // Initialize current state vector with altitude, velocity, and quaternion angles
  current_state[0] = bmp->readAltitude();
  current_state[1] = 0.0; // Velocity initialized to 0
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
    // Run through all the steps to update the current Kalman filter estimate of the state

    // Find the timestep dt
    curr_time = millis();
    dt = ((double)(curr_time - old_time) / 1000);
    old_time = curr_time;

    predict_state();
    predict_p_cov();
    update_gain();
    adjust_state();
    adjust_p_cov();

    // Update filtered altitude and velocity
    filteredAltitude = current_state[0];
    filteredVelocity = current_state[1];

    // Output results
    Serial.print("Filtered Altitude: ");
    Serial.println(filteredAltitude, 4);
    Serial.print("Filtered Velocity: ");
    Serial.println(filteredVelocity, 4);
}


void Kalman::predict_state() {
    // State prediction
    predicted_state[0] = (A[0][0] * current_state[0]) + (A[0][1] * current_state[1]) + (A[0][2] * current_state[2]);
    predicted_state[1] = current_state[1]; // Velocity remains constant
    predicted_state[2] = (A[2][0] * current_state[0]) + (A[2][1] * current_state[1]) + (A[2][2] * current_state[2]);
}

void Kalman::predict_p_cov() {
    // Used to predict the error/uncertainty in the state model
    // Target Equation: ApA' + Q

    // (pA')
    double B[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            B[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                B[i][j] += current_p_cov[i][k] * A[k][j];
            }
        }
    }

    // A*(pA') + Q
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            predicted_p_cov[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                predicted_p_cov[i][j] += A[i][k] * B[k][j];
            }
            predicted_p_cov[i][j] += Q[i][j];
        }
    }
}

void Kalman::update_gain() {
    // Update gain based on measurement noise and model noise
    // Target equation: pH' / (HpH' + R)

    // pH'
    double B[3][2];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            B[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                B[i][j] += predicted_p_cov[i][k] * H[k][j];
            }
        }
    }

    // H(pH' + R)^-1
    double C[2][2];
    double det = (predicted_p_cov[0][0] * predicted_p_cov[2][2]) - (predicted_p_cov[0][2] * predicted_p_cov[2][0]);
    C[0][0] = predicted_p_cov[2][2] / det;
    C[0][1] = -predicted_p_cov[0][2] / det;
    C[1][0] = -predicted_p_cov[2][0] / det;
    C[1][1] = predicted_p_cov[0][0] / det;

    // pH' * (HpH' + R)^-1
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            kalman_gain[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                kalman_gain[i][j] += B[i][k] * C[k][j];
            }
        }
    }
}

void Kalman::adjust_state() {
    // Update state with gain and new measurement
    // Target equation: x + k(m - Hx)

    // (m - Hx)
    double B[2];
    B[0] = bmp->readAltitude() - (H[0][0] * predicted_state[0]) - (H[0][1] * predicted_state[1]);
    B[1] = bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).z() - (H[1][0] * predicted_state[0]) - (H[1][1] * predicted_state[1]);

    // x + k(m - Hx)
    for (int i = 0; i < 3; i++) {
        adjusted_state[i] = predicted_state[i];
        for (int j = 0; j < 2; j++) {
            adjusted_state[i] += kalman_gain[i][j] * B[j];
        }
    }
}

void Kalman::adjust_p_cov() {
    // Target equation: p - kHp;
    // Hp
    double B[2][3];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            B[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                B[i][j] += H[i][k] * predicted_p_cov[k][j];
            }
        }
    }

    // p - kHp
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            adjusted_p_cov[i][j] = predicted_p_cov[i][j];
            for (int k = 0; k < 2; k++) {
                adjusted_p_cov[i][j] -= kalman_gain[i][k] * B[k][j];
            }
        }
    }
}

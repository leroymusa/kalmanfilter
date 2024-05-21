#include "KalmanFilter.h"

// State vector: [altitude, velocity, acceleration]
float state[3] = {0, 0, 0};

// Covariance matrix
float P[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};

// Process noise
float Q[3][3] = {
  {0.001, 0, 0},
  {0, 0.001, 0},
  {0, 0, 0.001}
};

// Measurement noise
float R_alt = 0.1; // Altitude measurement noise
float R_acc = 0.1; // Acceleration measurement noise

// Kalman gain
float K[3][2] = {0};

void kalmanInit() {
  // Initialization code if necessary
}

void kalmanPredict(float dt) {
  // State prediction
  state[0] += state[1] * dt + 0.5 * state[2] * dt * dt;
  state[1] += state[2] * dt;

  // Update covariance matrix
  float F[3][3] = {
    {1, dt, 0.5 * dt * dt},
    {0, 1, dt},
    {0, 0, 1}
  };

  float P_new[3][3] = {0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        P_new[i][j] += F[i][k] * P[k][j];
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P[i][j] = P_new[i][j] + Q[i][j];
    }
  }
}

void kalmanUpdate(float altitude, float accel) {
  // Measurement update
  float y[2] = {altitude - state[0], accel - state[2]};

  // Compute Kalman gain
  float S[2][2] = {
    {P[0][0] + R_alt, P[0][2]},
    {P[2][0], P[2][2] + R_acc}
  };
  
  float S_inv[2][2] = {
    {S[1][1], -S[0][1]},
    {-S[1][0], S[0][0]}
  };
  float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
  S_inv[0][0] /= det;
  S_inv[0][1] /= det;
  S_inv[1][0] /= det;
  S_inv[1][1] /= det;

  for (int i = 0; i < 3; i++) {
    K[i][0] = P[i][0] * S_inv[0][0] + P[i][2] * S_inv[1][0];
    K[i][1] = P[i][0] * S_inv[0][1] + P[i][2] * S_inv[1][1];
  }

  // Update state estimate
  for (int i = 0; i < 3; i++) {
    state[i] += K[i][0] * y[0] + K[i][1] * y[1];
  }

  // Update covariance matrix
  float I_KH[3][3] = {
    {1 - K[0][0], -K[0][1], 0},
    {-K[1][0], 1 - K[1][1], 0},
    {-K[2][0], -K[2][1], 1}
  };

  float P_new[3][3] = {0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        P_new[i][j] += I_KH[i][k] * P[k][j];
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P[i][j] = P_new[i][j];
    }
  }
}

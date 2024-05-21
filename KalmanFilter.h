#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// State vector: [altitude, velocity, acceleration]
extern float state[3];

// Covariance matrix
extern float P[3][3];

// Process noise
extern float Q[3][3];

// Measurement noise
extern float R_alt; // Altitude measurement noise
extern float R_acc; // Acceleration measurement noise

// Kalman gain
extern float K[3][2];

// Function declarations
void kalmanInit();
void kalmanPredict(float dt);
void kalmanUpdate(float altitude, float accel);

#endif

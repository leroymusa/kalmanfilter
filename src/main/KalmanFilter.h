class Kalman {
public:
  Kalman(float initialQ_angle, float initialQ_bias, float initialR_measure) {
    Q_angle = initialQ_angle;
    Q_bias = initialQ_bias;
    R_measure = initialR_measure;

    angle = 0;
    bias = 0;
    P[0][0] = 0;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0;
  }

  float getAngle(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }

  void setAngle(float newAngle) { angle = newAngle; }
  float getRate() { return rate; }

  void setQangle(float newQ_angle) { Q_angle = newQ_angle; }
  void setQbias(float newQ_bias) { Q_bias = newQ_bias; }
  void setRmeasure(float newR_measure) { R_measure = newR_measure; }

private:
  float Q_angle;
  float Q_bias;
  float R_measure;

  float angle;
  float bias;
  float rate;

  float P[2][2];
};

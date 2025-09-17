#include <Wire.h>

// ICM20948 I2C Address and Registers
#define ICM20948_ADDRESS 0x68
#define ACCEL_XOUT_H 0x2D
#define GYRO_XOUT_H 0x33

float ax, ay, az, gx, gy, gz;
float s_gx, s_gy, s_gz; // Starting gyro (degrees/sec) values - used for calibration

class Kalman_Angle_Filter{
  float angle = 0.0;         // Kalman filtered angle
  float bias = 0.0;          // Gyro bias
  float rate;                // Unbiased rate
  float P[2][2] = {{0, 0}, {0, 0}};  // Error covariance matrix
  
  const float Q_angle = 0.001; // Process noise variance for the angle
  const float Q_bias = 0.003;  // Process noise variance for the gyro bias
  const float R_measure = 0.03; // Measurement noise variance
  
  unsigned long lastTime = 0;
  
public:
 
  Kalman_Angle_Filter(float Q_angle, float Q_bias, float R_measure):
    Q_angle(Q_angle), Q_bias(Q_bias), R_measure(R_measure)
  {

  }

  float update(float new_angle, float new_rate);

};

Kalman_Angle_Filter Kalman_roll(0.001, 0.003, 0.03), Kalman_pitch(0.001, 0.003, 0.03);


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Starting ICM20948 initialization...");

  uint8_t whoAmI = readRegister(ICM20948_ADDRESS, 0x00); // WHO_AM_I register
  Serial.print("WHO_AM_I register: 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI != 0xEA) {
    Serial.println("ICM20948 not detected! Check connections and address.");
    while (1);
  }

  writeRegister(ICM20948_ADDRESS, 0x06, 0x01); // PWR_MGMT_1: Set clock source
  delay(500);

  readGyroscope(s_gx, s_gy, s_gz);
  Serial.print("STARTING DPS | x: ");
  Serial.print(s_gx);
  Serial.print(", y : ");
  Serial.print(s_gy);
  Serial.print(", z: ");
  Serial.println(s_gz);
  Serial.println("ICM20948 initialized successfully!");

}

void loop() {

  readAccelerometer(ax, ay, az);
  readGyroscope(gx, gy, gz);


  // --- Compute angles ---
  float acc_pitch = atan2(ay, az) * 180 / PI; // Approx pitch
  float gyro_rate = gx; // Use gyro X-axis for pitch in this example

  float kalman_pitch = Kalman_pitch.update(acc_pitch, gyro_rate);

  Serial.print("Accel pitch: ");
  Serial.print(acc_pitch);
  Serial.print("\tKalman pitch: ");
  Serial.println(kalman_pitch);

  // Serial.print("Accel (g): ");
  // Serial.print(ax); Serial.print(", ");
  // Serial.print(ay); Serial.print(", ");
  // Serial.println(az);

  // Serial.print("Gyro (dps): ");
  // Serial.print(gx - s_gx); Serial.print(", ");
  // Serial.print(gy - s_gy); Serial.print(", ");
  // Serial.println(gz - s_gz);

  // Serial.print(asin(constrain(ax, -1.0, 1.0)) * 180/M_PI); Serial.print(", ");
  // Serial.print(asin(constrain(ay, -1.0, 1.0)) * 180/M_PI); Serial.print(", ");
  // Serial.println(az);
  // Serial.print("Orientation: ");

  delay(500);
}

float Kalman_filter::update(float new_angle, float new_rate) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  // Predict
  rate = new_rate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Update
  float S = P[0][0] + R_measure;
  float K[2] = {P[0][0] / S, P[1][0] / S};

  float y = new_angle - angle;
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


void readAccelerometer(float &ax, float &ay, float &az) {
  int16_t rawX = (readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H) << 8) | readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H + 1);
  int16_t rawY = (readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H + 2) << 8) | readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H + 3);
  int16_t rawZ = (readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H + 4) << 8) | readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H + 5);

  ax = rawX / 16384.0; // Assuming default ±2g range
  ay = rawY / 16384.0;
  az = rawZ / 16384.0;
}

void readGyroscope(float &gx, float &gy, float &gz) {
  int16_t rawX = (readRegister(ICM20948_ADDRESS, GYRO_XOUT_H) << 8) | readRegister(ICM20948_ADDRESS, GYRO_XOUT_H + 1);
  int16_t rawY = (readRegister(ICM20948_ADDRESS, GYRO_XOUT_H + 2) << 8) | readRegister(ICM20948_ADDRESS, GYRO_XOUT_H + 3);
  int16_t rawZ = (readRegister(ICM20948_ADDRESS, GYRO_XOUT_H + 4) << 8) | readRegister(ICM20948_ADDRESS, GYRO_XOUT_H + 5);

  gx = rawX / 131.0; // Assuming default ±250 dps range
  gy = rawY / 131.0;
  gz = rawZ / 131.0;
}

uint8_t readRegister(uint8_t deviceAddress, uint8_t registerAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false);

  Wire.requestFrom(deviceAddress, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

void writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(value);
  Wire.endTransmission();
}
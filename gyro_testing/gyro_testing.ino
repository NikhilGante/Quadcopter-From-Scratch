#include <Wire.h>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

uint8_t readRegister(uint8_t deviceAddress, uint8_t registerAddress);
void writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value);

class Kalman_Angle_Filter {
  float angle = 0.0;         // Kalman filtered angle
  float bias = 0.0;          // Gyro bias
  float rate;                // Unbiased rate
  float P[2][2] = {{0, 0}, {0, 0}};  // Error covariance matrix

  const float Q_angle;
  const float Q_bias;
  const float R_measure;

  unsigned long lastTime = 0;

public:
  Kalman_Angle_Filter(float Q_angle_, float Q_bias_, float R_measure_)
    : Q_angle(Q_angle_), Q_bias(Q_bias_), R_measure(R_measure_) {}

  float update(float new_angle, float new_rate) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    // Serial.print("Delta time: ");
    // Serial.println(now - lastTime);
    lastTime = now;

    // Predict
    rate = new_rate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
    float S = P[0][0] + R_measure;
    float K[2] = { P[0][0] / S, P[1][0] / S };

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
};

  
// Simple IIR (Infinite Impulse Response) Exponential Filter
class Expo_Angle_Filter {
  float tau;             // Time constant in seconds (e.g. 0.5s) | larger -> smoother, smaller -> more responsive
  float angle = 0.0;
  unsigned long last_time = 0;
  bool initialized = false;

public:
  Expo_Angle_Filter(float tau): tau(tau) {}

  void init(float initial_angle) {
    angle = initial_angle;
    last_time = millis();
    initialized = true;
  }

  float update(float new_angle) {
    unsigned long now = millis();
    if (!initialized) {
      init(new_angle);
      return angle;
    }

    float dt = (now - last_time) / 1000.0f;  // Convert ms to seconds
    last_time = now;

    float alpha = 1.0f - exp(-dt / tau);  // Dynamic alpha

    angle = alpha * new_angle + (1.0f - alpha) * angle;
    return angle;
  }
};


class ICM20948_IMU {
  const int ICM20948_ADDRESS = 0x68;  // I2C Address
  const int ACCEL_XOUT_H = 0x2D, GYRO_XOUT_H = 0x33;  // Accel and Gyro Registers

  float s_gx, s_gy, s_gz; // Start (initial) gyro values - used for calibration

  float expo_roll, expo_pitch;

  // Kalman_Angle_Filter Kalman_roll = Kalman_Angle_Filter(0.000001, 0.005, 0.0001);
  // Kalman_Angle_Filter Kalman_pitch = Kalman_Angle_Filter(0.000001, 0.005, 0.0001);
  Expo_Angle_Filter Expo_roll = Expo_Angle_Filter(0.1), Expo_pitch = Expo_Angle_Filter(0.1);

public:
  float ax, ay, az, gx, gy, gz;
  void init() {
    Serial.println("Starting ICM20948 initialization...");

    uint8_t whoAmI = readRegister(ICM20948_ADDRESS, 0x00);
    Serial.print("WHO_AM_I register: 0x");
    Serial.println(whoAmI, HEX);

    if (whoAmI != 0xEA) {
      Serial.println("ICM20948 not detected! Check connections and address.");
      while (1);
    }

    writeRegister(ICM20948_ADDRESS, 0x06, 0x01); // PWR_MGMT_1: Set clock source
    delay(500);

    // readGyroscope(s_gx, s_gy, s_gz);
    // Serial.print("STARTING DPS | x: ");
    // Serial.print(s_gx);
    // Serial.print(", y : ");
    // Serial.print(s_gy);
    // Serial.print(", z: ");
    // Serial.println(s_gz);
    readAccelerometer(ax, ay, az);

    Expo_roll.init(atan2(ax, az) * RAD_TO_DEG);
    Expo_pitch.init(atan2(ay, az) * RAD_TO_DEG);

    Serial.println("ICM20948 initialized successfully!");
  }

  void update() {
    readAccelerometer(ax, ay, az);
    readGyroscope(gx, gy, gz);

    // kalman_roll = Kalman_roll.update(atan2(ax, az) * RAD_TO_DEG, gy);
    // kalman_pitch = Kalman_pitch.update(atan2(ay, az) * RAD_TO_DEG, gx);
    expo_roll = Expo_roll.update(atan2(ax, az) * RAD_TO_DEG);
    expo_pitch = Expo_pitch.update(atan2(ay, az) * RAD_TO_DEG);
    
  }

  void readAccelerometer(float &ax, float &ay, float &az);
  void readGyroscope(float &gx, float &gy, float &gz);

  float getRoll() const {
    return expo_roll;
  }

  float getPitch() const {
    return expo_pitch;
  }
};

// --- Global IMU object ---
ICM20948_IMU Imu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Imu.init();
}

void loop() {
  Imu.update();

  // Serial.print("Kalman Roll: ");
  Serial.print(Imu.getRoll());
  // Serial.print("\tPitch: ");
  Serial.print("\t");
  // Serial.print("Roll: ");

  Serial.println(atan2(Imu.ax, Imu.az) * RAD_TO_DEG);
  // Serial.println(Imu.getPitch());
  // Serial.println(Imu.getPitch());

}

// --- Sensor reading and I2C helpers ---

void ICM20948_IMU::readAccelerometer(float &ax, float &ay, float &az) {
  int16_t rawX = (readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H) << 8) | readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H + 1);
  int16_t rawY = (readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H + 2) << 8) | readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H + 3);
  int16_t rawZ = (readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H + 4) << 8) | readRegister(ICM20948_ADDRESS, ACCEL_XOUT_H + 5);

  ax = rawX / 16384.0;
  ay = rawY / 16384.0;
  az = rawZ / 16384.0;
}

void ICM20948_IMU::readGyroscope(float &gx, float &gy, float &gz) {
  int16_t rawX = (readRegister(ICM20948_ADDRESS, GYRO_XOUT_H) << 8) | readRegister(ICM20948_ADDRESS, GYRO_XOUT_H + 1);
  int16_t rawY = (readRegister(ICM20948_ADDRESS, GYRO_XOUT_H + 2) << 8) | readRegister(ICM20948_ADDRESS, GYRO_XOUT_H + 3);
  int16_t rawZ = (readRegister(ICM20948_ADDRESS, GYRO_XOUT_H + 4) << 8) | readRegister(ICM20948_ADDRESS, GYRO_XOUT_H + 5);

  gx = rawX / 131.0;
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

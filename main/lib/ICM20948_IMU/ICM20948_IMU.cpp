#include "ICM20948_IMU.h"
#include <Wire.h>
// General I2C read/write helper functions

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

// Simple IIR (Infinite Impulse Response) Exponential Filter
// Expo_Angle_Filter Methods

  void Expo_Angle_Filter::init(float initial_angle) {
    angle = initial_angle;
    last_time = millis();
    initialized = true;
  }

  float Expo_Angle_Filter::update(float new_angle) {
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


// ICM20948_IMU Methods

  void ICM20948_IMU::init(bool tare) {
    Serial.println("Starting ICM20948 initialization...");

    uint8_t whoAmI = readRegister(ICM20948_ADDRESS, 0x00);
    Serial.print("WHO_AM_I register: 0x");
    Serial.println(whoAmI, HEX);

    if (whoAmI != 0xEA) {
      Serial.println("ICM20948 not detected! Check connections and address.");
      while (1);
    }

    writeRegister(ICM20948_ADDRESS, 0x06, 0x01); // PWR_MGMT_1: Set clock source
    delay(500); // Wait for gyro to settle (stop moving)

    float gyro_sum_x = 0.0, gyro_sum_y = 0.0, gyro_sum_z = 0.0;
    for(int i = 0; i < 100; i += 1){
      readGyroscope(s_gx, s_gy, s_gz);
      gyro_sum_x += s_gx;
      gyro_sum_y += s_gy;
      gyro_sum_z += s_gz;
      delay(10);
    }
    s_gx = gyro_sum_x / 100.0;
    s_gy = gyro_sum_y / 100.0;
    s_gz = gyro_sum_z / 100.0;
/*
    Serial.print("STARTING DPS | x: ");
    Serial.print(s_gx);
    Serial.print(", y : ");
    Serial.print(s_gy);
    Serial.print(", z: ");
    Serial.println(s_gz);
*/
    readAccelerometer(ax, ay, az);

    
    start_roll_offset = filtered_roll = atan2(ax, az) * RAD_TO_DEG;
    start_pitch_offset = filtered_pitch = atan2(ay, az) * RAD_TO_DEG;
    
    if(tare){
      filtered_roll = 0.0;
      filtered_pitch = 0.0;
    }
    Serial.print("Initial roll offset: "); Serial.println(start_roll_offset);
    Serial.print("Initial pitch offset: "); Serial.println(start_pitch_offset); 
    Serial.print("Initial filtered roll: "); Serial.println(filtered_roll);
    Serial.print("Initial filtered pitch: "); Serial.println(filtered_pitch);

    Expo_filter_roll.init(filtered_roll);
    Expo_filter_pitch.init(filtered_pitch);

    Serial.println("ICM20948 initialized successfully!");

  }

  void ICM20948_IMU::update() {
    
    unsigned long now = millis();
    readAccelerometer(ax, ay, az);
    readGyroscope(gx, gy, gz);

    float dt = (now - last_time) / 1000.0f;  // Convert ms to seconds
    last_time = now;
    yaw += (gz - s_gz) * dt;
    roll += (gy - s_gy) * dt;

    float accel_roll = atan2(ax, az) * RAD_TO_DEG - start_roll_offset;
    // if(fabs(accel_roll - filtered_roll)/dt > 500.0){ // If angle changes more than x degs/s, listen only to gyro
    //   filtered_roll = Expo_filter_roll.update((filtered_roll + (gy - s_gy) * dt) * 0.5 + 0.5 * accel_roll);
    //   Serial.print("Relying only on gyro!\t");
    // }
    // else 
    filtered_roll = Expo_filter_roll.update(accel_roll);

    float accel_pitch = atan2(ay, az) * RAD_TO_DEG - start_pitch_offset;
    // if(fabs(accel_pitch - filtered_pitch)/dt > 500.0){ // If angle changes more than x degs/s, listen only to gyro
    //   filtered_pitch = Expo_filter_pitch.update((filtered_pitch + (gx - s_gx) * dt) * 0.5 + 0.5 * accel_pitch);
    //   Serial.print("[pitch]Relying only on gyro!\t");
    // }
    // else  
    filtered_pitch = Expo_filter_pitch.update(accel_pitch);
  }

  float ICM20948_IMU::getRoll() const {
    return filtered_roll;
  }

  float ICM20948_IMU::getPitch() const {
    return filtered_pitch;
  }

  float ICM20948_IMU::getYaw() const {
    return yaw;
  }

  float ICM20948_IMU::getYawRate() const{ // Returns in degrees/sec
    return gz - s_gz;
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


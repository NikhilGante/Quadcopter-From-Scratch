#pragma once

#include <Arduino.h>

uint8_t readRegister(uint8_t deviceAddress, uint8_t registerAddress);
void writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value);

// Simple IIR (Infinite Impulse Response) Exponential Filter
class Expo_Angle_Filter {
  float tau;             // Time constant in seconds (e.g. 0.5s) | larger -> smoother, smaller -> more responsive
  float angle = 0.0;
  unsigned long last_time = 0;
  bool initialized = false;

public:
  Expo_Angle_Filter(float tau): tau(tau) {}

  void init(float initial_angle);

  float update(float new_angle);
};

class ICM20948_IMU {
  const int ICM20948_ADDRESS = 0x68;  // I2C Address
  const int ACCEL_XOUT_H = 0x2D, GYRO_XOUT_H = 0x33;  // Accel and Gyro Registers

  float s_gx, s_gy, s_gz; // Start (initial) gyro values - used for calibration

  float filtered_roll = 0.0, filtered_pitch = 0.0;
  float roll = 0.0, pitch = 0.0, yaw = 0.0;
  unsigned long last_time = 0;

  Expo_Angle_Filter Expo_filter_roll = Expo_Angle_Filter(0.1), Expo_filter_pitch = Expo_Angle_Filter(0.1);

public:
  float ax, ay, az, gx, gy, gz;
  float start_roll_offset = 0.0, start_pitch_offset = 0.0;  // Angular offsets to be subtracted from accelerometer readings
  void init(bool tare = true);

  void update();    // To be called once every loop

  void readAccelerometer(float &ax, float &ay, float &az);
  void readGyroscope(float &gx, float &gy, float &gz);

  float getRoll() const;
  float getPitch() const;
  float getYaw() const;
  float getYawRate() const; // Returns in degrees/sec
};

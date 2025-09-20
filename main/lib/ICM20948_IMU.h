#pragma once

#include <Arduino.h>
#include <Wire.h>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

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

    float acc_sum = 0.0;
    for(int i = 0; i < 100; i += 1){
      readGyroscope(s_gx, s_gy, s_gz);
      acc_sum += s_gz;
      delay(10);
    }
    s_gz = acc_sum / 100.0;

    readAccelerometer(ax, ay, az);

    Expo_filter_roll.init(atan2(ax, az) * RAD_TO_DEG);
    Expo_filter_pitch.init(atan2(ay, az) * RAD_TO_DEG);

    Serial.println("ICM20948 initialized successfully!");
  }

  void update();    // To be called once every loop

  void readAccelerometer(float &ax, float &ay, float &az);
  void readGyroscope(float &gx, float &gy, float &gz);

  float getRoll() const;
  float getPitch() const;
  float getYaw() const;
  float getYawRate() const; // Returns in degrees/sec
};

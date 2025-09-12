#include <Wire.h>

// ICM20948 I2C Address and Registers
#define ICM20948_ADDRESS 0x68
#define ACCEL_XOUT_H 0x2D
#define GYRO_XOUT_H 0x33

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
  delay(10);

  Serial.println("ICM20948 initialized successfully!");
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  readAccelerometer(ax, ay, az);
  readGyroscope(gx, gy, gz);

  Serial.print("Accel (g): ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.println(az);

  Serial.print("Gyro (dps): ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);

  delay(500);
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
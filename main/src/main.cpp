#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>

// Local Includes
#include "ICM20948_IMU.h"

const int NWMotorPin = 3, NEMotorPin = 4, SWMotorPin = 6, SEMotorPin = 5;
const int throttlePin = 50, aileronPin = 46, elevatorPin = 48, rudderPin = 52;
int throttleVal, aileronVal, elevatorVal, rudderVal;

Servo NWMotor, NEMotor, SWMotor, SEMotor; // Create Servo objects to control the ESCs
float NWpwr, NEpwr, SWpwr, SEpwr;

const float max_allowed_pwr = 180;

double roll_input, roll_output, roll_setpoint;
double pitch_input, pitch_output, pitch_setpoint;

double Kp = 1.0, Ki = 0.0, Kd = 0.0;
PID Pid_roll(&roll_input, &roll_output, &roll_setpoint, Kp, Ki, Kd, DIRECT);
PID Pid_pitch(&pitch_input, &pitch_output, &pitch_setpoint, Kp, Ki, Kd, DIRECT);

ICM20948_IMU Imu;

void setup(){
  Serial.begin(115200);
  Wire.begin();

  // NWMotor.attach(NWMotorPin, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)
  // NEMotor.attach(NEMotorPin, 1000, 2000);
  // SWMotor.attach(SWMotorPin, 1000, 2000);
  // SEMotor.attach(SEMotorPin, 1000, 2000);

  Pid_roll.SetMode(AUTOMATIC);
  Pid_pitch.SetMode(AUTOMATIC);

  Imu.init();
}

void loop() {
  Imu.update();

  Serial.print("Kalman Roll: ");
  Serial.print(Imu.getRoll());
  Serial.print("\tPitch: ");
  // Serial.print("\t");
  Serial.print(Imu.getPitch());
  Serial.print("\tYaw: ");
  Serial.print(Imu.getYaw()); 

  // throttleVal = map(pulseIn(throttlePin, HIGH), 990, 1990, 0, 100);
  // aileronVal = map(pulseIn(aileronPin, HIGH), 995, 1985, -100, 100);
  // elevatorVal = map(pulseIn(elevatorPin, HIGH), 900, 1985, -100, 100);
  // rudderVal = map(pulseIn(rudderPin, HIGH), 950, 2080, -100, 100);

  throttleVal = map(pulseIn(throttlePin, HIGH), 990, 1990, 0, 180);
  aileronVal = map(pulseIn(aileronPin, HIGH), 995, 1985, -90, 90);
  elevatorVal = map(pulseIn(elevatorPin, HIGH), 900, 1985, -90, 90);
  rudderVal = map(pulseIn(rudderPin, HIGH), 950, 2080, -90, 90);

  // Serial.print("Throttle: "); Serial.print(throttleVal);
  // Serial.print("\tAileron: "); Serial.print(aileronVal);
  // Serial.print("\tElevator: "); Serial.print(elevatorVal);
  // Serial.print("\tRudder: "); Serial.println(rudderVal);  

  // Map to -30 - 30 degrees
  // roll_setpoint = aileronVal;
  // pitch_setpoint = elevatorVal;

  // Pid_roll.Compute();
  // Pid_pitch.Compute();

  // float max_pwr = throttleVal*0.5 + abs(aileronVal) * 0.3 + abs(elevatorVal) * 0.3 + rudderVal * 0.4;
  // NWpwr = throttleVal*0.5 + aileronVal*0.3 - elevatorVal*0.3 + rudderVal*0.4;
  // NEpwr = throttleVal*0.5 - aileronVal*0.3 - elevatorVal*0.3 - rudderVal*0.4;
  // SWpwr = throttleVal*0.5 + aileronVal*0.3 + elevatorVal*0.3 - rudderVal*0.4;
  // SEpwr = throttleVal*0.5 - aileronVal*0.3 + elevatorVal*0.3 + rudderVal*0.4;

  // // Scale all motor powers such that no motor exceeds max_allowed_pwr, while maintaining relative proportions 
  // if(max_pwr > max_allowed_pwr){  
  //   float scale = max_allowed_pwr/max_pwr;
  //   NWpwr *= scale;  
  //   NEpwr *= scale;  
  //   SWpwr *= scale;  
  //   SEpwr *= scale;  
  // }
  
  // NWMotor.write(NWpwr);
  // NEMotor.write(NEpwr);
  // SWMotor.write(SWpwr);
  // SEMotor.write(SEpwr);

  Serial.println();
}

#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>

const int NWMotorPin = 2, NEMotorPin = 3, SWMotorPin = 5, SEMotorPin = 6;
const int throttlePin = 7, aileronPin = 8, elevatorPin = 9, rudderPin = 10;
int throttleVal, aileronVal, elevatorVal, rudderVal;

Servo NWMotor, NEMotor, SWMotor, SEMotor; // Create Servo objects to control the ESCs
float NWpwr, NEpwr, SWpwr, SEpwr;

const float max_allowed_pwr = 180;

double roll_input, roll_output, roll_setpoint;
double pitch_input, pitch_output, pitch_setpoint;

double Kp = 1.0, Ki = 0.0, Kd = 0.0;
PID Pid_roll(&roll_input, &roll_output, &roll_setpoint, Kp, Ki, Kd, DIRECT);
PID Pid_pitch(&pitch_input, &pitch_output, &pitch_setpoint, Kp, Ki, Kd, DIRECT);

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup(){
  NWMotor.attach(NWMotorPin, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)
  NEMotor.attach(NEMotorPin, 1000, 2000);
  SWMotor.attach(SWMotorPin, 1000, 2000);
  SEMotor.attach(SEMotorPin, 1000, 2000);

  Pid_roll.SetMode(AUTOMATIC);
  Pid_pitch.SetMode(AUTOMATIC);

}

void loop() {
  throttleVal = map(pulseIn(throttlePin, HIGH), 1000, 2000, 0, 180);
  aileronVal = map(pulseIn(aileronPin, HIGH), 1000, 2000, -180, 180);
  elevatorVal = map(pulseIn(elevatorPin, HIGH), 1000, 2000, -180, 180);
  rudderVal = map(pulseIn(rudderPin, HIGH), 1000, 2000, -180, 180);

  // Map to -30 - 30 degrees
  roll_setpoint = aileronVal;
  pitch_setpoint = elevatorVal;

  Pid_roll.Compute();
  Pid_pitch.Compute();

  float max_pwr = throttleVal + abs(aileronVal) * 0.2 + abs(elevatorVal) * 0.2 + rudderVal * 0.2;
  NWpwr = throttleVal*0.5 + aileronVal*0.2 - elevatorVal*0.2 + rudderVal*0.2;
  NEpwr = throttleVal*0.5 - aileronVal*0.2 - elevatorVal*0.2 - rudderVal*0.2;
  SWpwr = throttleVal*0.5 + aileronVal*0.2 + elevatorVal*0.2 - rudderVal*0.2;
  SEpwr = throttleVal*0.5 - aileronVal*0.2 + elevatorVal*0.2 + rudderVal*0.2;

  // Scale all motor powers such that no motor exceeds max_allowed_pwr, while maintaining relative proportions 
  if(max_pwr > max_allowed_pwr){  
    float scale = max_allowed_pwr/max_pwr;
    NWpwr *= scale;  
    NEpwr *= scale;  
    SWpwr *= scale;  
    SEpwr *= scale;  
  }
  
  NWMotor.write(NWpwr);
  NEMotor.write(NEpwr);
  SWMotor.write(SWpwr);
  SEMotor.write(SEpwr);
}

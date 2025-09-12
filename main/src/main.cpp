#include <Arduino.h>
#include <Servo.h>
#include <PID.h>

const int NWMotorPin = 2, NEMotorPin = 3, SWMotorPin = 5, SEMotorPin = 6;
const int throttlePin = 7, aileronPin = 8, elevatorPin = 9, rudderPin = 10;
const int throttleVal, aileronVal, elevatorVal, rudderVal;

const float max_allowed_pwr = 180;

float NWpwr, NEpwr, SWpwr, SEpwr;

Servo NWMotor;     // create servo object to control the ESC
Servo NEMotor;
Servo SWMotor;
Servo SEMotor;

void setup() 
{
  NWMotor.attach(NWMotorPin, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)
  NEMotor.attach(NEMotorPin, 1000, 2000);
  SWMotor.attach(SWMotorPin, 1000, 2000);
  SEMotor.attach(SEMotorPin, 1000, 2000);
}

void loop() {
  throttleVal = map(pulseIn(throttlePin, HIGH), 1000, 2000, 0, 100);
  aileronVal = map(pulseIn(aileronPin, HIGH), 1000, 2000, -100, 100);
  elevatorVal = map(pulseIn(elevatorPin, HIGH), 1000, 2000, -100, 100);
  rudderVal = map(pulseIn(rudderPin, HIGH), 1000, 2000, -100, 100);

  float max_pwr = throttleVal + abs(aileronVal) * 0.2 + abs(elevatorVal) * 0.2 + rudderVal * 0.2;
  NWpwr = constrain(throttleVal*0.5 + aileronVal*0.2 - elevatorVal*0.2 + rudderVal*0.2, 5, max_allowed_pwr);
  NEpwr = constrain(throttleVal*0.5 - aileronVal*0.2 - elevatorVal*0.2 - rudderVal*0.2, 5, max_allowed_pwr);
  SWpwr = constrain(throttleVal*0.5 + aileronVal*0.2 + elevatorVal*0.2 - rudderVal*0.2, 5, max_allowed_pwr);
  SEpwr = constrain(throttleVal*0.5 - aileronVal*0.2 + elevatorVal*0.2 + rudderVal*0.2, 5, max_allowed_pwr);

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

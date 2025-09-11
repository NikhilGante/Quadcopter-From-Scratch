int NWMotorPin = 2;
int NEMotorPin = 3;
int SWMotorPin = 5;
int SEMotorPin = 6;

int throttlePin = 7;
int aileronPin = 8;
int elevatorPin = 9;
int rudderPin = 10;

int throttleVal;
int aileronVal;
int elevatorVal;
int rudderVal;

int motor1Check;
int motor2Check;
int motor3Check;
int motor4Check;

Servo NWMotor;     // create servo object to control the ESC
Servo NEMotor;
Servo SWMotor;
Servo SEMotor;

void setup() 
{
  NWMotor.attach(NWMotorPin,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  NEMotor.attach(NEMotorPin,1000,2000);
  SWMotor.attach(SWMotorPin,1000,2000);
  SEMotor.attach(SEMotorPin,1000,2000);
}

void loop() {
  throttleVal = map(pulseIn (throttlePin,HIGH), 1000, 2000, 0, 180);
  aileronVal = map(pulseIn (aileronPin, HIGH), 1000, 2000, -90, 90);
  elevatorVal = map(pulseIn (elevatorPin, HIGH), 1000, 2000, -90, 90);
  rudderVal = map(pulseIn (rudderPin, HIGH), 1000, 2000, -90, 90);
  motor1Check = ((throttleVal*0.5)+ (aileronVal+0.1)- (elevatorVal*0.25) + (rudderVal*0.15));
  motor2Check = ((throttleVal*0.5)- (aileronVal+0.1)- (elevatorVal*0.25) - (rudderVal*0.15));
  motor3Check = ((throttleVal*0.5)+ (aileronVal+0.1)+ (elevatorVal*0.25) - (rudderVal*0.15));
  motor4Check = ((throttleVal*0.5)- (aileronVal+0.1)+ (elevatorVal*0.25) + (rudderVal*0.15));

  if (motor1Check > 175)
  {motor1Check = 175;}
  else if (motor1Check < 5)
  {motor1Check = 5;}
  
  if (motor2Check > 175)
  {motor2Check = 175;}
  else if (motor2Check < 5)
  {motor2Check = 5;}
  
  if (motor3Check > 175)
  {motor3Check = 175;}
  else if (motor3Check < 5)
  {motor3Check = 5;}

  if (motor4Check > 175)
  {motor4Check = 175;}
  else if (motor4Check < 5)
  {motor4Check = 5;}
  
  NWMotor.write(motor1Check);
  NEMotor.write(motor2Check);
  SWMotor.write(motor3Check);
  SEMotor.write(motor4Check);
}

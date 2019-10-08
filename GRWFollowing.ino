#include <Servo.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "ConstrainedIntegrator.h"
#include "Integrator.h"

#define SETUP 0
#define IMUORULTRA 1
#define servoPin 7 // pin for servo signal
#define pingRightTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingRightEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingRightGrndPin 27 // ping sensor ground pin (use digital pin as ground)
#define pingFrontTrigPin 22 // ping sensor trigger pin (output from Arduino)
#define pingFrontEchoPin 24 // ping sensor echo pin (input to Arduino)
#define pingFrontGrndPin 26 // ping sensor ground pin (use digital pin as ground)
#define motorFwdPin 8 // HIGH for FWD; LOW for REV
#define motorRevPin 9 // LOW for FWD; HIGH for REV
#define motorLPWMPin 10 // Left Motor Speed Control
#define motorRPWMPin 11 // Right Motor Speed Control
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega
#define POT1PIN 7  //Pot pin
#define KPULTRA 10 //Kp ultrasonic
#define KIULTRA 0.3 //Ki ultrasonic
#define KPIMU 0.3 //Kp heading hold
#define FRAME_LENGTH 20000 //in us
#define GYRO_OFFSET 2.59 //Gyro bias
#define ACCEL_OFFSET -0.262 //Accelerometer bias
#define SHORTLEN 200 
#define LONGLEN 100

Servo steeringServo;
Adafruit_LSM9DS1 lsm;
int servo_middle = 1150;

void setup() {
  Serial.begin(115200);
  Serial.println("PI Test");
  lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);
  pinMode(pingRightGrndPin,OUTPUT); 
  digitalWrite(pingRightGrndPin,LOW);
  pinMode(pingRightTrigPin,OUTPUT);
  pinMode(pingRightEchoPin,INPUT);
  pinMode(pingFrontGrndPin,OUTPUT); 
  digitalWrite(pingFrontGrndPin,LOW);
  pinMode(pingFrontTrigPin,OUTPUT);
  pinMode(pingFrontEchoPin,INPUT);
  pinMode(motorFwdPin,OUTPUT); digitalWrite(motorFwdPin,HIGH);
  pinMode(motorRevPin,OUTPUT); digitalWrite(motorRevPin,LOW);
  pinMode(motorLPWMPin,OUTPUT); analogWrite(motorLPWMPin,0);
  pinMode(motorRPWMPin,OUTPUT); analogWrite(motorRPWMPin,0);
  steeringServo.attach(servoPin);

  if (!lsm.begin()) {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
    
  Serial.println("Found LSM9DS1 9DOF");
   
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS); 
}

void loop() {
  static double setup = 1;
  static double angle = 0;
  static double u_last = 0;
  static double ping_error = 0;
  double heading_error;
  static double heading_setpoint = 0;
  static double heading = 0;
  static byte motorLPWM = 100;
  static byte motorRPWM = 100;
  static float pingDistanceCMFront = 0.0;
  static float pingDistanceCMRight = 0.0;
  static double servoAngleDeg = 0.0;
  static int heading_init = 0;
  static double V = 58.8;
  static double distance_target = SHORTLEN;
  static ConstrainedIntegrator ping_integrator(0.0, -20.0, 20.0, FRAME_LENGTH*10e-6);
  static Integrator heading_integrator(0.0, FRAME_LENGTH*10e-6);
  static Integrator velocity_integrator(0.0, FRAME_LENGTH*10e-6);
  static Integrator distance_integrator(0.0, FRAME_LENGTH*10e-6);

  //Setup
  if (setup) {
    do {
        int val = analogRead(8);
        delay(100);
        Serial.println(val);
        servo_middle = 1150 - (val - 512)/4;
        setServoAngle(servoAngleDeg);
    } while (SETUP);
    setup = 0;
  }

  //Read IMU values
  lsm.read();  // ask it to read in the data
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  //Update Integrators
  velocity_integrator.new_value(a.acceleration.y - ACCEL_OFFSET);
  distance_integrator.new_value(V*FRAME_LENGTH*10e-6);
  heading_integrator.new_value(g.gyro.z-GYRO_OFFSET);

  //Emergency stop
  pingDistanceCMRight = getPingDistanceCM(0);
  digitalWrite(motorFwdPin,HIGH);
  digitalWrite(motorRevPin,LOW);
  pingDistanceCMFront = getPingDistanceCM(1);
  if (pingDistanceCMFront < 20.0) {
     analogWrite(motorLPWMPin,0);
     analogWrite(motorRPWMPin,0);
  } else {
     analogWrite(motorLPWMPin,motorLPWM);
     analogWrite(motorRPWMPin,motorRPWM);  
  }

  //Main logic for wall following
  if(distance_integrator.get_sum() < distance_target) {
    if (pingDistanceCMRight < 45) {
      ping_error = double(pingDistanceCMRight) - 30;
      ping_integrator.new_value(ping_error);
      
      servoAngleDeg = constrain(KPULTRA*ping_error + KIULTRA*ping_integrator.get_sum(), -20, 20);

      heading_init = 1;
    } else {
      if (heading_init) {
        heading_setpoint = heading;
        heading_init = 0;
      }
    
      heading_error = heading - heading_setpoint;
      servoAngleDeg = constrain(KPIMU*heading_error, -20, 20);
    }
  } else {
    distance_integrator.reset(0);
    heading_setpoint = 0;
    distance_target = LONGLEN;
  }

  setServoAngle(servoAngleDeg);

  //Ensures fixed frame length
  if (micros() - u_last > FRAME_LENGTH) {
    Serial.println(micros() - u_last);
    Serial.println("Frame overrun");  
  }
  
  while (micros() - u_last < FRAME_LENGTH) {
    
  }
  u_last = micros();
}

// Ping sensor measurement
double getPingDistanceCM(int choice)
{
  int pingTrig = -1;
  int pingEcho = -1;
  if (choice == 0) {
    pingTrig = pingRightTrigPin;
    pingEcho = pingRightEchoPin;  
  } else {
    pingTrig = pingFrontTrigPin;
    pingEcho = pingFrontEchoPin;
  }

  const long timeout_us = 3000;
  
  digitalWrite(pingTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrig, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTrig, LOW);
  
  unsigned long echo_time;
  echo_time = pulseIn(pingEcho, HIGH, timeout_us);
  if (echo_time == 0)
  {
    echo_time = timeout_us;
  }
 
  return constrain(0.017*echo_time,5.0,50.0);

}

//Set servo
void setServoAngle(double sDeg)
{
    double ServoCenter_us = servo_middle;
    double ServoScale_us = 8.0;    // micro-seconds per degree

    double t_us = constrain(ServoCenter_us + ServoScale_us * sDeg, ServoCenter_us-150, ServoCenter_us+150);
    steeringServo.writeMicroseconds(t_us);
}

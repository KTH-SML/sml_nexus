#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "sml_nexus_motor.h"

// Var definition


unsigned long prevUpdateTime;

//UR wheel motor
int intCount1 = 0;
#define MOTOR1_ENC_A 3
#define MOTOR1_ENC_B 13
nexusMotor URMotor(8, 9);
//Adafruit_DCMotor *URMotor = AFMS.getMotor(1);

//LR wheel motor
int intCount2 = 0;
#define MOTOR2_ENC_A 19
#define MOTOR2_ENC_B A12
//Adafruit_DCMotor *LRMotor = AFMS.getMotor(2);
nexusMotor LRMotor(10, 11);

//LL wheel motor
int intCount3 = 0;
#define MOTOR3_ENC_A 18
#define MOTOR3_ENC_B A14
//Adafruit_DCMotor *LLMotor = AFMS.getMotor(3);
nexusMotor LLMotor(5, 4);

//UL wheel motor
int intCount4 = 0;
#define MOTOR4_ENC_A 2
#define MOTOR4_ENC_B 12
//Adafruit_DCMotor *ULMotor = AFMS.getMotor(4);
nexusMotor ULMotor(6, 7);


ros::NodeHandle nh;


std_msgs :: Float32MultiArray meas_msg;

ros::Publisher meas_speed("velocity", &meas_msg);

const int encoder_CPR = 1536;
const float wheel_radius = 0.05;  //Wheel radius (in m)

float ULspeed = 0;
float URspeed = 0;
float LLspeed = 0;
float LRspeed = 0;

float control_signal;

float measUL, measUR, measLL, measLR;

int L1 = 150;
int L2 = 150;

double long updateRate = 50000.0; // uS
double long vx, vy, w;


float errorUL = 0;
float errorUR = 0;
float errorLL = 0;
float errorLR = 0;

int pwmUL = 0;
int pwmUR = 0;
int pwmLL = 0;
int pwmLR = 0;

void pwmSubCb(const std_msgs::Float32MultiArray& msg){
  pwmLL = (int)msg.data[0];
  pwmLR = (int)msg.data[1];
  pwmUL = (int)msg.data[2];
  pwmUR = (int)msg.data[3];
}

static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

ros::Subscriber<std_msgs::Float32MultiArray> pwm_sub("cmd_pwm", &pwmSubCb );

void setup() {
  // put your setup code here, to run once:
  TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR4B = TCCR4B & B11111000 | B00000001;
  TCCR3B = TCCR3B & B11111000 | B00000001;    // set timer 3 divisor to     1 for PWM frequency of 31372.55 Hz
  
  vx = 0;
  vy = 0;
  w = 0;

  ULspeed = 0;
  URspeed = 0;
  LLspeed = 0;
  LRspeed = 0;
  
  pinMode(MOTOR1_ENC_A, INPUT);
  pinMode(MOTOR1_ENC_B, INPUT);
  
  pinMode(MOTOR2_ENC_A, INPUT);
  pinMode(MOTOR2_ENC_B, INPUT);
  
  pinMode(MOTOR3_ENC_A, INPUT);
  pinMode(MOTOR3_ENC_B, INPUT);
  
  pinMode(MOTOR4_ENC_A, INPUT);
  pinMode(MOTOR4_ENC_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_A), encoderM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_A), encoderM2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_ENC_A), encoderM3A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR4_ENC_A), encoderM4A, CHANGE);

  meas_msg.data_length = 4;
  meas_msg.data = (float*)malloc(sizeof(float)*4);

  nh.initNode();
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(pwm_sub);
  nh.advertise(meas_speed);

  prevUpdateTime = 0;
}

void loop() {
  double long now = micros();
  float updateOldness = now - prevUpdateTime;
  
  if(updateOldness >= updateRate)
  {
    //Serial.print("\nEncoder ticks:");
    //Serial.print(intCount1);

    //Compute speed:  Get rads from tick increments       convert to rad/s      |v=wr| convert to m/s
    measLR = ((float)intCount3/1536)*(2*3.1415) * ((float)1000000/updateOldness) * wheel_radius;
    intCount3 = 0;
    measUR = ((float)intCount4/1536)*(2*3.1415) * ((float)1000000/updateOldness) * wheel_radius;
    intCount4 = 0;
    measUL = ((float)intCount1/1536)*(2*3.1415) * ((float)1000000/updateOldness) * wheel_radius;
    intCount1 = 0;
    measLL = ((float)intCount2/1536)*(2*3.1415) * ((float)1000000/updateOldness) * wheel_radius;
    intCount2 = 0;

    //Set motor speeds
    LRMotor.setSpeed(pwmLR);
    LLMotor.setSpeed(pwmLL);
    ULMotor.setSpeed(pwmUL);
    URMotor.setSpeed(pwmUR);

    // reset vars
    prevUpdateTime = now;  

    meas_msg.data[0] = measUL;
    meas_msg.data[1] = measUR;
    meas_msg.data[2] = measLL;
    meas_msg.data[3] = measLR;
    meas_speed.publish(&meas_msg);

    nh.spinOnce();
  }
  nh.spinOnce();
    
}

// Interrupts
void encoderM1A(){
  if (digitalRead(MOTOR1_ENC_A) == digitalRead(MOTOR1_ENC_B)) ++intCount1;
  else --intCount1;
}

void encoderM2A(){
  if (digitalRead(MOTOR2_ENC_A) == digitalRead(MOTOR2_ENC_B)) ++intCount2;
  else --intCount2;
}

void encoderM3A(){
  if (digitalRead(MOTOR3_ENC_A) == digitalRead(MOTOR3_ENC_B)) --intCount3;
  else ++intCount3;
}

void encoderM4A(){
  if (digitalRead(MOTOR4_ENC_A) == digitalRead(MOTOR4_ENC_B)) --intCount4;
  else ++intCount4;
}

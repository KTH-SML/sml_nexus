/*
Library for the nexus 4WD holonomic robot.

TO BE USED ON ARDUINO MEGA

# Author : Robin Baran
# Date   : 2020.6.17
# Ver    : 1

*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <PID_v1.h>
#include "sml_nexus_motor.h"

/******************** Variables ****************/

unsigned long prevUpdateTime;
double long updateOldness;
double long now;

//UR wheel motor
int intCount3 = 0;
#define MOTOR3_ENC_A 18
#define MOTOR3_ENC_B A14
nexusMotor URMotor(8, 9);

//LR wheel motor
int intCount4 = 0;
#define MOTOR4_ENC_A 2
#define MOTOR4_ENC_B 12
nexusMotor LRMotor(10, 11);

//LL wheel motor
int intCount1 = 0;
#define MOTOR1_ENC_A 3
#define MOTOR1_ENC_B 13
nexusMotor LLMotor(5, 44);

//UL wheel motor
int intCount2 = 0;
#define MOTOR2_ENC_A 19
#define MOTOR2_ENC_B A12
nexusMotor ULMotor(7, 6);

double outputPIDUL = 0;
double outputPIDUR = 0;
double outputPIDLL = 0;
double outputPIDLR = 0;

float control_signal;

double measUL, measUR, measLL, measLR;

double L1 = 150;
double L2 = 150;

double long lastReceivedCommTimeout;
double long commTimeout = 500; // ms
double long updateRate = 50.0; // ms
double long vx, vy, w;

float errorUL = 0;
float errorUR = 0;
float errorLL = 0;
float errorLR = 0;

int pwmUL = 0;
int pwmUR = 0;
int pwmLL = 0;
int pwmLR = 0;

double ULspeed = 0;
double URspeed = 0;
double LLspeed = 0;
double LRspeed = 0;

int test;

double polyCmdUL = 0;
double polyCmdUR = 0;
double polyCmdLL = 0;
double polyCmdLR = 0;

/************ Robot-specific constants ************/
const int encoder_CPR = 1536;
const float wheel_radius = 0.05;  //Wheel radius (in m)
const double speed_to_pwm_ratio = 120;     //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command.

double max_speed = 0.5; //max speed per wheel in m/s
double min_speed = 0.005; //minimum that will stop the motor command if reached, in m/s

//-----------------------------------
// Setup a PID object for each wheel
//-----------------------------------
// PID Parameters, respectively Kp, Ki and Kd
float PID_default_params[] = { 5.0, 0.0, 0.0 };
// Feedforward default gain
float feedForwardPolyDefault[] = { 21, 115, 1200, -2000, 1200 };

float PID_UL_params[3];
float PID_UR_params[3];
float PID_LL_params[3];
float PID_LR_params[3];

float feedForwardPolyUL[5];
float feedForwardPolyUR[5];
float feedForwardPolyLL[5];
float feedForwardPolyLR[5];

//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction) 
PID PID_UL(&measUL, &outputPIDUL, &ULspeed, PID_default_params[0], PID_default_params[1], PID_default_params[2], DIRECT);
PID PID_UR(&measUR, &outputPIDUR, &URspeed, PID_default_params[0], PID_default_params[1], PID_default_params[2], DIRECT);
PID PID_LL(&measLL, &outputPIDLL, &LLspeed, PID_default_params[0], PID_default_params[1], PID_default_params[2], DIRECT);
PID PID_LR(&measLR, &outputPIDLR, &LRspeed, PID_default_params[0], PID_default_params[1], PID_default_params[2], DIRECT);


////-------------------------------
//// PWM command callback function
////-------------------------------
//void pwmSubCb(const std_msgs::Float32MultiArray& msg){
//  pwmUL = (int)msg.data[0];
//  pwmUR = (int)msg.data[1];
//  pwmLL = (int)msg.data[2];
//  pwmLR = (int)msg.data[3];
//}




/************ Velocity command callback function ************/
void messageCb( const geometry_msgs::Twist& msg){
  vx = (float)msg.linear.x;
  vy = (float)msg.linear.y;
  w = (float)msg.angular.z;

  lastReceivedCommTimeout = millis() + commTimeout;
}



/************ PID tuning callback function ************/
void pidCb( const std_msgs :: Float32MultiArray& msg){
//Receives Kp, Ki and Kd for UL, UR, LL and LR respectively
  //if message has length 12
//  if (msg.data_lenght == 12){
//    PID_UL.SetTunings(msg.data[0], msg.data[1], msg.data[2]);
//    PID_UR.SetTunings(msg.data[3], msg.data[4], msg.data[5]);
//    PID_LL.SetTunings(msg.data[6], msg.data[7], msg.data[8]);
//    PID_LR.SetTunings(msg.data[9], msg.data[10], msg.data[11]);
//  }
}




/************ Sign function ************/
static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}



//------------------------------------
// Setup ROS publishers & subscribers
//------------------------------------
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &messageCb );
//ros::Subscriber<std_msgs :: Float32MultiArray> pid_sub("pid_tuning", &pidCb );
std_msgs :: Float32MultiArray meas_msg;
ros::Publisher measuredVelPub("velocity", &meas_msg);
//std_msgs :: Float32MultiArray output_msg;
//std_msgs :: Float32MultiArray pwm_msg;
//ros::Publisher output_pub("output", &output_msg);
//ros::Publisher pwm_pub("pwm", &pwm_msg);



/************ PID parameters and setup function ************/
void setupPIDParams(){
  //------------------------------------------------------------------------------------
  // Get PID parameters from ROS parameter server. If not available, set default params
  //------------------------------------------------------------------------------------
  int count = 0;
  //nh.spinOnce();
//  if(!nh.getParam("test", &test)){
//    nh.logwarn("TEST FAILED;");
//  }
//  else{
//    nh.logwarn("TEST OK;");
//  }
  if(!nh.getParam("PID_UL", PID_UL_params, 3, 300)){
    PID_UL_params[0] = PID_default_params[0];
    PID_UL_params[1] = PID_default_params[1];
    PID_UL_params[2] = PID_default_params[2];
    nh.logwarn("UL motor PID: loading default values;");
  }

  if(!nh.getParam("feedforward_UL", feedForwardPolyUL, 5, 300)){
    feedForwardPolyUL[0] = feedForwardPolyDefault[0];
    feedForwardPolyUL[1] = feedForwardPolyDefault[1];
    feedForwardPolyUL[2] = feedForwardPolyDefault[2];
    feedForwardPolyUL[3] = feedForwardPolyDefault[3];
    feedForwardPolyUL[4] = feedForwardPolyDefault[4];
    nh.logwarn("UL motor feedforward gain: loading default values;");
  }

  if(!nh.getParam("PID_UR", PID_UR_params, 3, 300)){
    PID_UR_params[0] = PID_default_params[0];
    PID_UR_params[1] = PID_default_params[1];
    PID_UR_params[2] = PID_default_params[2];
    nh.logwarn("UR motor PID: loading default values;");
  }

  if(!nh.getParam("feedforward_UR", feedForwardPolyUR, 5, 300)){
    feedForwardPolyUR[0] = feedForwardPolyDefault[0];
    feedForwardPolyUR[1] = feedForwardPolyDefault[1];
    feedForwardPolyUR[2] = feedForwardPolyDefault[2];
    feedForwardPolyUR[3] = feedForwardPolyDefault[3];
    feedForwardPolyUR[4] = feedForwardPolyDefault[4];
    nh.logwarn("UR motor feedforward gain: loading default values;");
  }

  if(!nh.getParam("PID_LL", PID_LL_params, 3, 300)){
    PID_LL_params[0] = PID_default_params[0];
    PID_LL_params[1] = PID_default_params[1];
    PID_LL_params[2] = PID_default_params[2];
    nh.logwarn("LL motor PID: loading default values;");
  }

  if(!nh.getParam("feedforward_LL", feedForwardPolyLL, 5, 300)){
    feedForwardPolyLL[0] = feedForwardPolyDefault[0];
    feedForwardPolyLL[1] = feedForwardPolyDefault[1];
    feedForwardPolyLL[2] = feedForwardPolyDefault[2];
    feedForwardPolyLL[3] = feedForwardPolyDefault[3];
    feedForwardPolyLL[4] = feedForwardPolyDefault[4];
    nh.logwarn("LL motor feedforward gain: loading default values;");
  }
  if(!nh.getParam("PID_LR", PID_LR_params, 3, 300)){
    PID_LR_params[0] = PID_default_params[0];
    PID_LR_params[1] = PID_default_params[1];
    PID_LR_params[2] = PID_default_params[2];
    nh.logwarn("LR motor PID: loading default values;");
  }

  if(!nh.getParam("feedforward_LR", feedForwardPolyLR, 5, 300)){
    feedForwardPolyLR[0] = feedForwardPolyDefault[0];
    feedForwardPolyLR[1] = feedForwardPolyDefault[1];
    feedForwardPolyLR[2] = feedForwardPolyDefault[2];
    feedForwardPolyLR[3] = feedForwardPolyDefault[3];
    feedForwardPolyLR[4] = feedForwardPolyDefault[4];
    nh.logwarn("LR motor feedforward gain: loading default values;");
  }
    
  //------------------------
  // Setting PID parameters
  //------------------------
  PID_UL.SetTunings(PID_UL_params[0], PID_UL_params[1], PID_UL_params[2]);
  PID_UR.SetTunings(PID_UR_params[0], PID_UR_params[1], PID_UR_params[2]);
  PID_LL.SetTunings(PID_LL_params[0], PID_LL_params[1], PID_LL_params[2]);
  PID_LR.SetTunings(PID_LR_params[0], PID_LR_params[1], PID_LR_params[2]);
  PID_UL.SetSampleTime(50);
  PID_UR.SetSampleTime(50);
  PID_LL.SetSampleTime(50);
  PID_LR.SetSampleTime(50);
  PID_UL.SetOutputLimits(-200, 200);
  PID_UR.SetOutputLimits(-200, 200);
  PID_LL.SetOutputLimits(-200, 200);
  PID_LR.SetOutputLimits(-200, 200);
  PID_UL.SetMode(AUTOMATIC);
  PID_UR.SetMode(AUTOMATIC);
  PID_LL.SetMode(AUTOMATIC);
  PID_LR.SetMode(AUTOMATIC);

}



/************ Setup topics over ROS ************/
void setupCommonTopics(){
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_sub);
 // nh.subscribe(pid_sub);
  nh.advertise(measuredVelPub);
 //nh.advertise(output_pub);
  //nh.advertise(pwm_pub);
}



/************ Get wheel velocity commands ************
              from main velocity command            */  
void computeWheelVelCmd(){
  //===================================
  // Map vx, vy, w to each wheel speed 
  //===================================
  ULspeed = constrain(1*vx - 1*vy - sqrt(sq(L1)+sq(L2))*w, -max_speed, max_speed);
  URspeed = constrain(1*vx + 1*vy + sqrt(sq(L1)+sq(L2))*w, -max_speed, max_speed);
  LLspeed = constrain(1*vx + 1*vy - sqrt(sq(L1)+sq(L2))*w, -max_speed, max_speed);
  LRspeed = constrain(1*vx - 1*vy + sqrt(sq(L1)+sq(L2))*w, -max_speed, max_speed);
}



/************ Get wheel velocities from encoders  ************/
void getWheelVel(){
  //Compute speed:  Get rads from tick increments       convert to rad/s      |v=wr| convert to m/s
  measUR = ((float)intCount3/1536)*(2*3.1415) * ((float)1000/updateOldness) * wheel_radius;
  intCount3 = 0;
  measLR = ((float)intCount4/1536)*(2*3.1415) * ((float)1000/updateOldness) * wheel_radius;
  intCount4 = 0;
  measLL = ((float)intCount1/1536)*(2*3.1415) * ((float)1000/updateOldness) * wheel_radius;
  intCount1 = 0;
  measUL = ((float)intCount2/1536)*(2*3.1415) * ((float)1000/updateOldness) * wheel_radius;
  intCount2 = 0;
}



/************ Compute motor inputs from feedforward and PID  ************/
void computeMotorInputs(){
  //--------------------------------------------------
  //   For each motor, check if velocity command is
  // larger than minimum speed and compute PID output
  //
  //      Feedforward is a 6th degree polynomial
  //    matching the PWM/Velocity response of each
  //                      motor
  //--------------------------------------------------
  //For UL wheel motor
  if (abs(ULspeed) > min_speed){
    // Compute feedforward polynomial command
    polyCmdUL = 0;
    for(int i=0; i<5; i++){
      polyCmdUL += feedForwardPolyUL[i]*pow((float)ULspeed,i);
    }
    // Compute PID output
    PID_UL.Compute();
    pwmUL = (int)polyCmdUL + (int)outputPIDUL;
    // Constrain to minimum PWM command
    //if (ULspeed > 0) pwmUL = constrain( pwmUL, 10, 200 );
    //else             pwmUL = constrain( pwmUL, -200, -10 );
  }
  //--------
  //For UR wheel motor
  if (abs(URspeed) > min_speed){
    // Compute feedforward polynomial command
    polyCmdUR = 0;
    for(int i=0; i<5; i++){
      polyCmdUR += feedForwardPolyUR[i]*pow((float)URspeed,i);
    }
    // Compute PID output
    PID_UR.Compute();
    pwmUR = (int)polyCmdUR + (int)outputPIDUR;
    // Constrain to minimum PWM command
    //if (URspeed > 0) pwmUR = constrain( pwmUR, 10, 200 );
    //else             pwmUR = constrain( pwmUR, -200, -10 );
  }
  //---------
  //For LL wheel motor
  if (abs(LLspeed) > min_speed){
    // Compute feedforward polynomial command
    polyCmdLL = 0;
    for(int i=0; i<5; i++){
      polyCmdLL += feedForwardPolyLL[i]*pow((float)LLspeed,i);
    }
    // Compute PID output
    PID_LL.Compute();
    pwmLL = (int)polyCmdLL + (int)outputPIDLL;
    // Constrain to minimum PWM command
    //if (LLspeed > 0) pwmLL = constrain( pwmLL, 10, 200 );
    //else             pwmLL = constrain( pwmLL, -200, -10 );
  }
  //---------
  //For LR wheel motor
  if (abs(LRspeed) > min_speed){
    // Compute feedforward polynomial command
    polyCmdLR = 0;
    for(int i=0; i<5; i++){
      polyCmdLR += feedForwardPolyLR[i]*pow((float)LRspeed,i);
    }
    // Compute PID output
    PID_LR.Compute();
    pwmLR = (int)polyCmdLR + (int)outputPIDLR;
    // Constrain to minimum PWM command
    //if (LRspeed > 0) pwmLR = constrain( pwmLR, 10, 200 );
    //else             pwmLR = constrain( pwmLR, -200, -10 );
  }       
}



/************ Apply previously computed motor command  ************/
void applyMotorInputs(){
  //Set motor speeds
  LRMotor.setSpeed(pwmLR);
  LLMotor.setSpeed(pwmLL);
  ULMotor.setSpeed(pwmUL);
  URMotor.setSpeed(pwmUR);
}



/************ Encoders interrupt functions ************/
void encoderLL(){
  if (digitalRead(MOTOR1_ENC_A) == digitalRead(MOTOR1_ENC_B)) --intCount1;
  else ++intCount1;
}

void encoderUL(){
  if (digitalRead(MOTOR2_ENC_A) == digitalRead(MOTOR2_ENC_B)) --intCount2;
  else ++intCount2;
}

void encoderUR(){
  if (digitalRead(MOTOR3_ENC_A) == digitalRead(MOTOR3_ENC_B)) ++intCount3;
  else --intCount3;
}

void encoderLR(){
  if (digitalRead(MOTOR4_ENC_A) == digitalRead(MOTOR4_ENC_B)) ++intCount4;
  else --intCount4;
}

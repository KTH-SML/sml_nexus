#include <ros.h>

/************ ROS Node Handle ************/
ros::NodeHandle  nh;

#include "sml_nexus_common.h"
#include "sml_nexus_ultrasonic_sensors.h"

void setup() {
  vx = 0;
  vy = 0;
  w = 0;

  //----------------
  // Pin mode setup
  //----------------
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

  //-----------------------------------------
  // Start ROS node, publishers & subscribers
  //------------------------------------------
  //Init measured speed message
  meas_msg.data_length = 4;
  meas_msg.data = (float*)malloc(sizeof(float)*4);
  //output_msg.data_length = 4;
  //output_msg.data = (float*)malloc(sizeof(float)*4);
  //pwm_msg.data_length = 4;
  //pwm_msg.data = (float*)malloc(sizeof(float)*4);
  nh.initNode();

//  //wait until connected with rosserial before continuing
//  while( !nh.connected() ){
//    nh.spinOnce();
//  }
  delay(1500);
  
//    //Wait for topics to initialize
//  int count = 0;
//  while (count < 100){
//    nh.spinOnce();
//    delay(10);
//    count++;
//  }

  //Advertise common topics over ROS
  setupCommonTopics();

  //Setup sensor messages and advertise sensor topics over ROS
  setupSensorTopics();
  
//  //Wait for topics to initialize
//  int count = 0;
//  while (count < 100){
//    nh.spinOnce();
//    delay(10);
//    count++;
//  }

  //Start adafruit motor shield
  AFMS.begin();
  
  prevUpdateTime = 0;
  lastReceivedCommTimeout = - commTimeout; //Ensure timeout at initialization
  
  //Setup the wheel velocity PIDs
  setupPIDParams();

}

void loop() {
  //#################################
  //
  //       Sensor reading loop
  //
  //#################################
  
  //==========================
  // Run sensor communication
  //==========================
  runSensor();

  //#################################
  //
  //            Main loop
  //
  //#################################
  now = millis();
  updateOldness = now - prevUpdateTime;
  if (updateOldness >= updateRate)
  {
    
    //--------------------------
    // Reset PWM command values 
    //--------------------------
    pwmUL = 0;
    pwmUR = 0;
    pwmLL = 0;
    pwmLR = 0;
    outputPIDUL = 0;
    outputPIDUR = 0;
    outputPIDLL = 0;
    outputPIDLR = 0;
    
    //===================================
    // Map vx, vy, w to each wheel speed 
    //===================================
    computeWheelVelCmd();

    //=====================================
    // Measure current speed of each wheel
    //=====================================
    getWheelVel();

    //==========================================
    // If command received recently, run motors
    //==========================================
    if (now < lastReceivedCommTimeout)
    { 
      computeMotorInputs();
    }
    
    //=====================
    // Apply motor command
    //=====================
    applyMotorInputs();
      
    // reset vars
    prevUpdateTime = now;  

    //===============================
    // Populate and publish messages
    //===============================
    // Populate messages
    //pwm_msg.data[0] = pwmUL;
    //pwm_msg.data[1] = pwmUR;
    //pwm_msg.data[2] = pwmLL;
    //pwm_msg.data[3] = pwmLR;
    // output_msg.data[0] = polyCmdUL;
    // output_msg.data[1] = polyCmdUR;
    // output_msg.data[2] = polyCmdLL;
    // output_msg.data[3] = polyCmdLR;
    meas_msg.data[0] = measUL;
    meas_msg.data[1] = measUR;
    meas_msg.data[2] = measLL;
    meas_msg.data[3] = measLR;
    // Publish message
    measuredVelPub.publish(&meas_msg);
//    // output_pub.publish(&output_msg);
//    // pwm_pub.publish(&pwm_msg);
//
  } 
  nh.spinOnce();
}

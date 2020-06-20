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
//  output_msg.data_length = 4;
//  output_msg.data = (float*)malloc(sizeof(float)*4);
  pwm_msg.data_length = 4;
  pwm_msg.data = (float*)malloc(sizeof(float)*4);
 
  nh.initNode();

  //wait until connected with rosserial before continuing
  while( !nh.connected() ){
    nh.spinOnce();
  }

  //Setup the wheel velocity PIDs
  setupPIDParams();
  
  //Advertise common topics over ROS
  setupCommonTopics();

  //Setup sensor messages and advertise sensor topics over ROS
  setupSensorTopics();

  //Wait for topics to initialize
  int count = 0;
  while (count < 100){
    nh.spinOnce();
    delay(10);
    count++;
  }

  prevUpdateTime = 0;
  lastReceivedCommTimeout = - commTimeout; //Ensure timeout at initialization
  
}

void loop() {

  //#################################
  //
  //       Sensor reading loop
  //
  //#################################
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
    ULspeed = constrain(1*vx - 1*vy - sqrt(sq(L1)+sq(L2))*w, -max_speed, max_speed);
    URspeed = constrain(1*vx + 1*vy + sqrt(sq(L1)+sq(L2))*w, -max_speed, max_speed);
    LLspeed = constrain(1*vx + 1*vy - sqrt(sq(L1)+sq(L2))*w, -max_speed, max_speed);
    LRspeed = constrain(1*vx - 1*vy + sqrt(sq(L1)+sq(L2))*w, -max_speed, max_speed);

   
    //=====================================
    // Measure current speed of each wheel
    //=====================================
    //Compute speed:  Get rads from tick increments       convert to rad/s      |v=wr| convert to m/s
    measLR = ((double)intCount3/1536)*(2*3.1415) * ((double)1000000/updateOldness) * wheel_radius;
    intCount3 = 0;
    measUR = ((float)intCount4/1536)*(2*3.1415) * ((float)1000000/updateOldness) * wheel_radius;
    intCount4 = 0;
    measUL = ((float)intCount1/1536)*(2*3.1415) * ((float)1000000/updateOldness) * wheel_radius;
    intCount1 = 0;
    measLL = ((float)intCount2/1536)*(2*3.1415) * ((float)1000000/updateOldness) * wheel_radius;
    intCount2 = 0;

    //==========================================
    // If command received recently, run motors
    //==========================================
    if (now < lastReceivedCommTimeout)
    { 
      //--------------------------------------------------
      //   For each motor, check if velocity command is
      // larger than minimum speed and compute PID output
      //--------------------------------------------------
      //For UL wheel motor
      if (abs(ULspeed) > min_speed){
        // Compute PID output
        PID_UL.Compute();
        pwmUL = feedForwardGainUL*ULspeed + outputPIDUL;
        // Constrain to minimum PWM command
        //if (ULspeed > 0) pwmUL = constrain( pwmUL, 10, 200 );
        //else             pwmUL = constrain( pwmUL, -200, -10 );
      }
      //--------
      //For UR wheel motor
      if (abs(URspeed) > min_speed){
        // Compute PID output
        PID_UR.Compute();
        pwmUR = feedForwardGainUR*URspeed + outputPIDUR;
        // Constrain to minimum PWM command
        //if (URspeed > 0) pwmUR = constrain( pwmUR, 10, 200 );
        //else             pwmUR = constrain( pwmUR, -200, -10 );
      }
      //---------
      //For LL wheel motor
      if (abs(LLspeed) > min_speed){
        // Compute PID output
        PID_LL.Compute();
        pwmLL = feedForwardGainLL*LLspeed + outputPIDLL;
        // Constrain to minimum PWM command
        //if (LLspeed > 0) pwmLL = constrain( pwmLL, 10, 200 );
        //else             pwmLL = constrain( pwmLL, -200, -10 );
      }
      //---------
      //For LR wheel motor
      if (abs(LRspeed) > min_speed){
        // Compute PID output
        PID_LR.Compute();
        pwmLR = feedForwardGainLR*LRspeed + outputPIDLR;
        // Constrain to minimum PWM command
        //if (LRspeed > 0) pwmLR = constrain( pwmLR, 10, 200 );
        //else             pwmLR = constrain( pwmLR, -200, -10 );
      }       
    }
    
    //=====================
    // Apply motor command
    //=====================
    //Set motor direction
    //Set motor speeds
    if (pwmUL>0) ULMotor->run(FORWARD);
    else if (pwmUL<0) ULMotor->run(BACKWARD);
    else ULMotor->run(RELEASE);
    if (pwmUR>0) URMotor->run(FORWARD);
    else if (pwmUR<0) URMotor->run(BACKWARD);
    else URMotor->run(RELEASE);
    if (pwmLL>0) LLMotor->run(FORWARD);
    else if (pwmLL<0) LLMotor->run(BACKWARD);
    else LLMotor->run(RELEASE);
    if (pwmLR>0) LRMotor->run(FORWARD);
    else if (pwmLR<0) LRMotor->run(BACKWARD);
    else LRMotor->run(RELEASE);
    
    LRMotor->setSpeed(abs(pwmLR));
    LLMotor->setSpeed(abs(pwmLL));
    ULMotor->setSpeed(abs(pwmUL));
    URMotor->setSpeed(abs(pwmUR));
      
    // reset vars
    prevUpdateTime = now;  

    //===============================
    // Populate and publish messages
    //===============================
    // Populate messages
    pwm_msg.data[0] = pwmUL;
    pwm_msg.data[1] = pwmUR;
    pwm_msg.data[2] = pwmLL;
    pwm_msg.data[3] = pwmLR;
//    output_msg.data[0] = outputPIDUL;
//    output_msg.data[1] = outputPIDUR;
//    output_msg.data[2] = outputPIDLL;
//    output_msg.data[3] = outputPIDLR;
    meas_msg.data[0] = measUL;
    meas_msg.data[1] = measUR;
    meas_msg.data[2] = measLL;
    meas_msg.data[3] = measLR;
    // Publish message
    meas_speed.publish(&meas_msg);
//    output_pub.publish(&output_msg);
    pwm_pub.publish(&pwm_msg);
   
  } 
    nh.spinOnce();
}

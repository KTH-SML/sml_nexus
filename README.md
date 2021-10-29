# SML Nexus
Repositotry for code and documentation on the customized Nexus 4WD holonomic robots of the SML (Smart Mobility Lab) of KTH.

<img src="/resources/4wd_holonomic_robot.png" align="center" width="600"/>

# Using the Nexus Robot
## Bringing up the Nexus robot
Every Nexus robot includes a companion computer (usually a NVidia TX2) connected to the internal Arduino low-level controller. To use the Nexus, one needs first to connect by SSH to the companion computer (credentials available by asking a lab member, replace COMPANION_COMPUTER_IP by the companion computer IP):

 `ssh sml@COMPANION_COMPUTER_IP`
 
Then the ROS_MASTER_URI and ROS_IP environnement variables need to be set accordingly. Check [ROS network setup ](http://wiki.ros.org/ROS/NetworkSetup) for more information.

To bring up the robot, run the following launch file from the companion computer:

 `roslaunch sml_nexus_robot sml_nexus_bringup.launch`
 
 After a successful launch, command and feedback topics should be available under the namespace **/nexus_ROBOT_ID**
 
## Sending commands
The robot can be commanded in velocity by publishing [ROS Twist message](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) on the topic **/nexus_ROBOT_ID/cmd_vel**

## Receiving feedback
TODO
 
# Robot description
The robot is a modified 4 mecanum wheel holonomic robot from Nexus Robotics (https://www.nexusrobot.com/product/4wd-mecanum-wheel-mobile-arduino-robotics-car-10011.html).

<img src="/resources/nexus_architecture.png" align="center" width="900"/>

## Characteristics
* **max speed:** 0.6m/s forward
* **wheel base:** 300mm

## Actuators
 * **Motors:** (4x) Faulhaber 12V DC Coreless Motor 16002, with 45° 50mm radius mecanum wheel. 
 * **Optional manipulator:** Either a **Robotis ViperX 250** or a **Robotis WidowX** depending on the platform.

## Controllers
 * **Onboard computer:** Either **NVidia TX2**, **NVidia Jetson Nano** or **Intel NUC** depending on the platform.
 * **Low-level controller:** Arduino Mega for interfacing with motor drivers, ultrasonic range sensor and encoders.
 * **Motor drivers:** Two Cytron MDD3A motor drivers.
 
## Sensors
* **(4x) Encoders:** Encoder on each wheel with a resolution of 12CPR before motor gearbox and 768CPR after gearbox.
* **(4x) Ultrasonic range sensor:** URM04 v1.0 ultrasonic range sensors with RS-485 interface.

# ROS package description
Packages:
* **sml_nexus_description**
* **sml_nexus_robot**

## sml_nexus_description
Contains the robot URDF description and meshes.

### Launch files
* **sml_nexus_description.launch:** Load the SML nexus 4WD mecanum robot description parameter and the robot state publisher.

* **sml_nexus_rviz.launch:** Load the SML nexus 4WD mecanum robot description parameter and a RViz session.

## sml_nexus_robot
Package to be run from the robot onboard computer.
### Launch files
* **sml_nexus_bringup.launch:** Load config files and connect to the low-level controller using rosserial.

### Config files
* **nexus_pid_params.yaml** Parameters of the motor controllers

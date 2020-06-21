# SML Nexus
Repositotry for code and documentation on the customized Nexus 4WD holonomic robots of the SML (Smart Mobility Lab) of KTH.

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

 
# Robot description


# Package description


# Setting up a new robot
### Hardeware
### Software

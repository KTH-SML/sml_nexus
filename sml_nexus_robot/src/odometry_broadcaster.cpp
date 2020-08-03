#include <ros/ros.h>
#include <cmath>
#include <ros/time.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class SmlNexusOdometryBroadcaster
{
public:
	SmlNexusOdometryBroadcaster();
	~SmlNexusOdometryBroadcaster();
private:
	void wheelVelCallback();
	void publishOdometry();
	void computeOdometry(const float ULWheelVel,
						 const float URWheelVel,
					     const float LLWheelVel,
						 const float LRWheelVel,
						 geometry_msgs::Pose prev_pose, 
					 	 ros::Time time_interval);

	geometry_msgs::Twist computeVel(const float ULWheelVel,
									const float URWheelVel,
									const float LLWheelVel,
									const float LRWheelVel);

	tf::Transform computeRelativeMotion();

	//ROS variables
	//=============
	void setSubAndPub();
	std::string ns; //Parameters namespace
	//Subscriber and publishers
	ros::Subscriber vel_sub;
	ros::Publisher odometry_pub;
	static tf2_ros::TransformBroadcaster transform_broadcaster;

	//
	float robot_wheelbase = 0.15; //robot wheelbase in meters
	ros::Time last_received_data;
	nav_msgs::Odometry odom_msg;
	geometry_msgs::TransformStamped odom_transform;
	geometry_msgs::Pose global_pose;

}

//=====================
// 	   constructor
//=====================
SmlNexusOdometryBroadcaster(){
	ros::NodeHandle nh;
	ns = nh.getNamespace();

	ROS_INFO(ns+" Starting odometry broadcaster");

	//Initialize the robot wheelbase radius used later on
	const float robot_wheelbase_sq = robot_wheelbase * robot_wheelbase;
	robot_wheelbase_radius = std::sqrt(2 * robot_wheelbase_sq);

	//Setup ROS subscribers and publishers
	setSubAndPub();
}


//=======================================
//   Setup ROS subscribers, publishers
//        and pre-fill messages
//=======================================
void setSubAndPub(){
	ROS_INFO(ns+" odometry broadcaster: setting up publishers and subscribers...");

	//------------------------------------------
	// Setup wheel velocity feedback subscriber
	//------------------------------------------
	feedback_sub = nh_.subscribe("wheel_velocity", 1000, &wheelVelCallback, this);

	//--------------------------
	// Setup odometry publisher
	//--------------------------
	cmd_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1000);

	//-----------------------------
	// Initialize odometry message
	//-----------------------------
	odom_msg.header.frame_id = ns+"/odom";
	odom_msg.child_frame_id = ns+"/base_link";

	//------------------------------
	// Initialize transform message
	//------------------------------
	odom_transform.header.frame_id = ns+"/odom";
	odom_transform.child_frame_id = ns+"/base_link";

}


wheelVelCallback(const std_msgs::Float32MultiArray& msg){
	if (msg.data.size() != 4){
		//Exit if msg is malformed
	}
	else{
		ros::Time time_interval = ros::Time::now() - last_received_data;
		odom_msg = computeOdometry(msg.data[0], msg.data[1], msg.data[2], msg.data[3], global_pose, time_interval);
	}
	


}

void computeOdometry(const float ULWheelVel,
					 const float URWheelVel,
					 const float LLWheelVel, 
					 const float LRWheelVel, 
					 geometry_msgs::Pose prev_pose, 
					 ros::Time time_interval)
{
	odom_msg.twist.twist = computeVel(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);

	tf::Transform tmp;
    tmp.setIdentity();


    if (std::abs(odom_msg.twist.twist.angular.z) < 0.0001) {
    //----------------
    // Drive straight
    //----------------
      tmp.setOrigin(tf::Vector3(static_cast<double>(odom_msg.twist.twist.linear.x*timeSeconds), static_cast<double>(odom_msg.twist.twist.linear.y*timeSeconds), 0.0));
    } else {
    //---------------------
    // Follow circular arc
    //---------------------
      const double distX = odom_msg.twist.twist.linear.x * timeSeconds;
      const double distY = odom_msg.twist.twist.linear.y * timeSeconds;
      const double distChange = std::sqrt(distX * distX + distY * distY);
      const double angleDriveDirection = std::acos(distX / distChange);
      const double angleChange = odom_msg.twist.twist.angular.z * timeSeconds;

      const double arcRadius = distChange / angleChange;

      tf::Vector3 endPos = tf::Vector3(std::sin(angleChange) * arcRadius,
                                arcRadius - std::cos(angleChange) * arcRadius,
                                0.0);

      tmp.setOrigin(endPos.rotate(tf::Vector3(0.0, 0.0, 1.0), angleDriveDirection));

      tmp.setRotation(tf::createQuaternionFromYaw(angleChange));
    }

}

//======================================
//	Compute velocity in base link frame
//======================================
geometry_msgs::Twist computeVel(const float ULWheelVel, const float URWheelVel, const float LLWheelVel, const float LRWheelVel){
	geometry_msgs::Twist relativeVel;

	relativeVel.linear.x = (ULWheelVel + URWheelVel + LLWheelVel + LRWheelVel) / 4;
	relativeVel.linear.y = (- ULWheelVel + URWheelVel + LLWheelVel - LRWheelVel) / 4;
	relativeVel.linear.z = 0.0;

	relativeVel.angular.x = 0.0;
	relativeVel.angular.y = 0.0;
	relativeVel.angular.z = (- ULWheelVel + URWheelVel - LLWheelVel + LRWheelVel) / (8 * robot_wheelbase);

	return relativeVel;
}

computeRelativeMotion()


//=============================
//			  Main
//=============================
int main(int argc, char** argv){
	ros::init(argc, argv, "odometry_node");
	
	try{
		SmlNexusOdometryBroadcaster odometry_broadcaster;
		ros::spin();
	}
	//Error handling
	catch (int error){
		if (error == 1){
			ROS_FATAL("Node can't initialize, failed to get parameters");
		}
		else{
			ROS_FATAL("Node encountered an unexpected error");		
		}
		return 1;
	}
	return 0;
}
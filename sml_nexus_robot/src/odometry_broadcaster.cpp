#include <ros/ros.h>
#include <cmath>
#include <ros/time.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class SmlNexusOdometryBroadcaster
{
public:
    SmlNexusOdometryBroadcaster();
    ~SmlNexusOdometryBroadcaster();
private:
    void wheelVelCallback(const std_msgs::Float32MultiArray& msg);
    void computeOdometry(nav_msgs::Odometry& odom,
                         const float& ULWheelVel,
                         const float& URWheelVel,
                         const float& LLWheelVel, 
                         const float& LRWheelVel, 
                         ros::Duration& time_interval);

    geometry_msgs::Twist computeVel(const float& ULWheelVel,
                                    const float& URWheelVel,
                                    const float& LLWheelVel,
                                    const float& LRWheelVel);

    nav_msgs::Odometry computeRelativeMotion(const float& ULWheelVel,
                                             const float& URWheelVel,
                                             const float& LLWheelVel,
                                             const float& LRWheelVel,
                                             const float& timeSeconds);

    //ROS variables
    //=============
    void setSubAndPub(ros::NodeHandle& nh_);
    std::string ns; //Parameters namespace
    //Subscriber and publishers
    ros::Subscriber feedback_sub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster transform_broadcaster;

    //
    float robot_wheelbase = 0.15; //robot wheelbase in meters
    ros::Time last_received_data;
    ros::Time time_now;
    bool init = false;
    nav_msgs::Odometry odom_msg;
    geometry_msgs::TransformStamped odom_transform;

};

//=====================
//        constructor
//=====================
SmlNexusOdometryBroadcaster::SmlNexusOdometryBroadcaster(){
    ros::NodeHandle nh;
    ns = nh.getNamespace();

    ROS_INFO_STREAM(ns << " Starting odometry broadcaster");

    //Setup ROS subscribers and publishers
    setSubAndPub(nh);
}

SmlNexusOdometryBroadcaster::~SmlNexusOdometryBroadcaster(){}

//=======================================
//   Setup ROS subscribers, publishers
//        and pre-fill messages
//=======================================
void SmlNexusOdometryBroadcaster::setSubAndPub(ros::NodeHandle& nh_){
    ROS_INFO_STREAM(ns << " odometry broadcaster: setting up publishers and subscribers...");

    //------------------------------------------
    // Setup wheel velocity feedback subscriber
    //------------------------------------------
    feedback_sub = nh_.subscribe("wheel_velocity", 1000, &SmlNexusOdometryBroadcaster::wheelVelCallback, this);

    //--------------------------
    // Setup odometry publisher
    //--------------------------
    odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1000);

    //-----------------------------
    // Initialize odometry message
    //-----------------------------
    odom_msg.header.frame_id = ns+"odom";
    odom_msg.child_frame_id = ns+"base_link";
    odom_msg.pose.pose.orientation.w = 1; //unit quaternion

    //------------------------------
    // Initialize transform message
    //------------------------------
    odom_transform.header.frame_id = ns+"odom";
    odom_transform.child_frame_id = ns+"base_link";

}


void SmlNexusOdometryBroadcaster::wheelVelCallback(const std_msgs::Float32MultiArray& msg){
    if (msg.data.size() != 4){
        //Exit if msg is malformed
    }
    else if (!init){
        //If first message do nothing but init last received data time
        last_received_data = ros::Time::now();
        init = true;
    }
    else{
        time_now = ros::Time::now();
        ros::Duration time_interval = time_now - last_received_data;
        ROS_INFO_STREAM("Time interval:" << std::endl << time_interval << std::endl);
        computeOdometry(odom_msg, msg.data[0], msg.data[1], msg.data[2], msg.data[3], time_interval);

        //Publish odometry
        odom_msg.header.stamp = time_now;
        odom_pub.publish(odom_msg);

        //Publish transform
        odom_transform.transform.translation.x = odom_msg.pose.pose.position.x;
        odom_transform.transform.translation.y = odom_msg.pose.pose.position.y;
        odom_transform.transform.translation.z = odom_msg.pose.pose.position.z;
        odom_transform.transform.rotation = odom_msg.pose.pose.orientation;
        transform_broadcaster.sendTransform(odom_transform);

        last_received_data = time_now;
    }
    


}

void SmlNexusOdometryBroadcaster::computeOdometry(nav_msgs::Odometry& odom,
                     const float& ULWheelVel,
                     const float& URWheelVel,
                     const float& LLWheelVel, 
                     const float& LRWheelVel, 
                     ros::Duration& time_interval)
{   
    const nav_msgs::Odometry relativeMotion = computeRelativeMotion(ULWheelVel, URWheelVel, LLWheelVel, LRWheelVel, time_interval.toSec());
    ROS_INFO_STREAM("Relation motion: " << std::endl << relativeMotion);
    tf2::Quaternion q_prev, q_rot, q_new, new_translation;
    tf2::Vector3 rel_translation;

    ROS_INFO_STREAM("Previous orientation: " << std::endl << odom.pose.pose.orientation << std::endl);

    //Convert to quaternion type for computation
    tf2::convert(odom.pose.pose.orientation, q_prev);
    tf2::convert(relativeMotion.pose.pose.orientation, q_rot);
    q_new = q_rot * q_prev;
    ROS_INFO_STREAM("q_new : " << std::endl << q_new.x() << ", " << q_new.y() << ", " << q_new.z() << ", " << q_new.w() << std::endl);

    q_new.normalize();
    tf2::convert(q_new, odom.pose.pose.orientation);

    ROS_INFO_STREAM("q_prev : " << std::endl << q_prev.x() << ", " << q_prev.y() << ", " << q_prev.z() << ", " << q_prev.w() << std::endl);
    ROS_INFO_STREAM("q_rot : " << std::endl << q_rot.x() << ", " << q_rot.y() << ", " << q_rot.z() << ", " << q_rot.w() << std::endl);
    ROS_INFO_STREAM("q_new_normalized : " << std::endl << q_new.x() << ", " << q_new.y() << ", " << q_new.z() << ", " << q_new.w() << std::endl);

    //Rotate relative translation to odometry frame of reference
    tf2::convert(relativeMotion.pose.pose.position, rel_translation);
    new_translation = (inverse(q_prev) * rel_translation) * q_prev; //using inverse of q_prev to rotate relative translation to odometry frame

    odom.pose.pose.position.x = odom.pose.pose.position.x + new_translation.x();
    odom.pose.pose.position.y = odom.pose.pose.position.y + new_translation.y();
    odom.pose.pose.position.z = odom.pose.pose.position.z + new_translation.z();

    odom.twist = relativeMotion.twist;
}

//======================================
//    Compute velocity in base link frame
//======================================
geometry_msgs::Twist SmlNexusOdometryBroadcaster::computeVel(const float& ULWheelVel, const float& URWheelVel, const float& LLWheelVel, const float& LRWheelVel){
    geometry_msgs::Twist relativeVel;

    relativeVel.linear.x = (ULWheelVel + URWheelVel + LLWheelVel + LRWheelVel) / 4;
    relativeVel.linear.y = (- ULWheelVel + URWheelVel + LLWheelVel - LRWheelVel) / 4;
    relativeVel.linear.z = 0.0;

    relativeVel.angular.x = 0.0;
    relativeVel.angular.y = 0.0;
    relativeVel.angular.z = (- ULWheelVel + URWheelVel - LLWheelVel + LRWheelVel) / (8 * robot_wheelbase);

    return relativeVel;
}

nav_msgs::Odometry SmlNexusOdometryBroadcaster::computeRelativeMotion(const float& ULWheelVel, const float& URWheelVel, const float& LLWheelVel, const float& LRWheelVel, const float& timeSeconds){
    ROS_INFO_STREAM(std::endl << "TIME SECONDS: " << timeSeconds << std::endl);
    nav_msgs::Odometry odom;
    const geometry_msgs::Twist vel = computeVel(ULWheelVel, URWheelVel, LLWheelVel, LRWheelVel);
    odom.twist.twist = vel;

    double angleChange = 0.0;

    if (std::abs(vel.angular.z) < 0.0001) {
        //----------------
        // Drive straight
        //----------------
        odom.pose.pose.position.x = static_cast<double>(vel.linear.x*timeSeconds);
        odom.pose.pose.position.y = static_cast<double>(vel.linear.y*timeSeconds);
        odom.pose.pose.position.z = 0.0;

    } else {
        //---------------------
        // Follow circular arc
        //---------------------
        const double distX = vel.linear.x * timeSeconds;
        const double distY = vel.linear.y * timeSeconds;
        const double distChange = std::sqrt(distX * distX + distY * distY);
        const double angleDriveDirection = std::acos(distX / distChange);
        angleChange = vel.angular.z * timeSeconds;

        const double arcRadius = distChange / angleChange;

        tf::Vector3 endPos = tf::Vector3(std::sin(angleChange) * arcRadius,
                                        arcRadius - std::cos(angleChange) * arcRadius,
                                        0.0);

        endPos = endPos.rotate(tf::Vector3(0.0, 0.0, 1.0), angleDriveDirection);

        odom.pose.pose.position.x = endPos[0];
        odom.pose.pose.position.y = endPos[1];
        odom.pose.pose.position.z = 0.0;

        

    }
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(angleChange), odom.pose.pose.orientation);
    return odom;
}


//==============================
//             Main
//==============================
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
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

    void runOdometry(const std_msgs::Float32MultiArray& msg_,
                     const float& time_interval_sec_,
                     const ros::Time& time_stamp);

    void computeOdometry(nav_msgs::Odometry& odom,
                         const float& ULWheelVel,
                         const float& URWheelVel,
                         const float& LLWheelVel, 
                         const float& LRWheelVel, 
                         const float& time_interval_seconds);

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
    if (ns == "/") ns = "";

    ROS_INFO_STREAM(ns << "Odometry broadcaster: startup...");

    //Setup ROS subscribers and publishers
    setSubAndPub(nh);
}

SmlNexusOdometryBroadcaster::~SmlNexusOdometryBroadcaster(){}

//=======================================
//   Setup ROS subscribers, publishers
//        and pre-fill messages
//=======================================
void SmlNexusOdometryBroadcaster::setSubAndPub(ros::NodeHandle& nh_){
    ROS_INFO_STREAM(ns << "Odometry broadcaster: setting up publishers and subscribers...");

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
        float time_interval_sec = time_interval.toSec();

        if (time_interval_sec < 2.0){
            runOdometry(msg, time_interval_sec, time_now);
            last_received_data = time_now;
        }
        else{
            ROS_WARN_STREAM(ns << "Odometry broadcaster: last data received " << time_interval_sec << " sec ago, odometry might lose accuracy");
            last_received_data = time_now;
        }
    }
    
}

void SmlNexusOdometryBroadcaster::runOdometry(const std_msgs::Float32MultiArray& msg_, const float& time_interval_sec_, const ros::Time& time_stamp){
        computeOdometry(odom_msg, msg_.data[0], msg_.data[1], msg_.data[2], msg_.data[3], time_interval_sec_);

        //Publish odometry
        odom_msg.header.stamp = time_stamp;
        odom_pub.publish(odom_msg);

        //Publish transform
        odom_transform.transform.translation.x = odom_msg.pose.pose.position.x;
        odom_transform.transform.translation.y = odom_msg.pose.pose.position.y;
        odom_transform.transform.translation.z = odom_msg.pose.pose.position.z;
        odom_transform.transform.rotation = odom_msg.pose.pose.orientation;
        transform_broadcaster.sendTransform(odom_transform);

}

void SmlNexusOdometryBroadcaster::computeOdometry(nav_msgs::Odometry& odom,
                     const float& ULWheelVel,
                     const float& URWheelVel,
                     const float& LLWheelVel, 
                     const float& LRWheelVel, 
                     const float& time_interval_seconds)
{   
    const nav_msgs::Odometry relativeMotion = computeRelativeMotion(ULWheelVel, URWheelVel, LLWheelVel, LRWheelVel, time_interval_seconds);
    ROS_INFO_STREAM("Relative motion " << std::endl << relativeMotion << std::endl);
    tf2::Quaternion q_prev, q_rot, q_new, new_translation;
    tf2::Vector3 rel_translation;

    //Convert to quaternion type for computation
    tf2::convert(odom.pose.pose.orientation, q_prev);
    tf2::convert(relativeMotion.pose.pose.orientation, q_rot);
    q_new = q_rot * q_prev;

    q_new.normalize();
    tf2::convert(q_new, odom.pose.pose.orientation);

    //Rotate relative translation to odometry frame of reference
    tf2::convert(relativeMotion.pose.pose.position, rel_translation);
    new_translation = (q_prev * rel_translation) * q_prev.inverse(); //rotate relative translation to odometry frame

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
    nav_msgs::Odometry rel_motion;
    const geometry_msgs::Twist vel = computeVel(ULWheelVel, URWheelVel, LLWheelVel, LRWheelVel);
    rel_motion.twist.twist = vel;

    double angleChange = 0.0;

    if (std::abs(vel.angular.z) < 0.0001) {
        //----------------
        // Drive straight
        //----------------
        rel_motion.pose.pose.position.x = static_cast<double>(vel.linear.x*timeSeconds);
        rel_motion.pose.pose.position.y = static_cast<double>(vel.linear.y*timeSeconds);
        rel_motion.pose.pose.position.z = 0.0;

    } else {
        //---------------------
        // Follow circular arc
        //---------------------
        const double distX = vel.linear.x * timeSeconds;
        const double distY = vel.linear.y * timeSeconds;
        const double distChange = std::sqrt(distX * distX + distY * distY);
        angleChange = vel.angular.z * timeSeconds;


        if (distChange == 0){
            //Rotating on the spot
            rel_motion.pose.pose.position.x = 0.0;
            rel_motion.pose.pose.position.y = 0.0;
            rel_motion.pose.pose.position.z = 0.0;
        }
        else{
            const double angleDriveDirection = std::atan(distY / distX);

            const double arcRadius = distChange / angleChange;

            tf::Vector3 endPos = tf::Vector3(std::sin(angleChange) * arcRadius,
                                            arcRadius - std::cos(angleChange) * arcRadius,
                                            0.0);

            endPos = endPos.rotate(tf::Vector3(0.0, 0.0, 1.0), angleDriveDirection);

            rel_motion.pose.pose.position.x = endPos[0];
            rel_motion.pose.pose.position.y = endPos[1];
            rel_motion.pose.pose.position.z = 0.0;
        }

    }
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(angleChange), rel_motion.pose.pose.orientation);
    return rel_motion;
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
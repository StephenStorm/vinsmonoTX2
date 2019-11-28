#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sstream>
#include <iostream>


//#include <math>

using namespace std;

/**
 * This file converts the odometry measurements from the dataset into a ground truth path
 */
nav_msgs::Path path; 
ros::Publisher path_pub;
double offsets[4] = {0};
bool first_time = true;
tf2::Quaternion q_orig, q_rot, q_new;
tf2::Quaternion quat_tf, quat_offset_tf;
geometry_msgs::Quaternion quat_msg;
int seq_offset;
double roll, pitch, yaw;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	// shift path to start at origin
	if (first_time) 
	{
		// translation offsets
		offsets[0] = msg->pose.pose.position.x;
		offsets[1] = msg->pose.pose.position.y;
		offsets[2] = msg->pose.pose.position.z;
		
		// find yaw offset
		geometry_msgs::Quaternion quat_msg = msg->pose.pose.orientation;
		
		// convert recevied geometry_message quat to tf quat
		tf2::convert(quat_msg , quat_tf);
		
		// get roll pitch yaw from t_f
		tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
		
		// store yaw offset in quat_offset_tf
		quat_offset_tf.setRPY(0, 0, -yaw);
		quat_offset_tf.normalize();
		
		ROS_INFO("Yaw offset: [%f]", yaw);
		first_time = false;
	}

    geometry_msgs::PoseStamped newPose;
    newPose.pose = msg->pose.pose;
    newPose.header = msg->header;
    
    newPose.pose.position.x = newPose.pose.position.x - offsets[0];
    newPose.pose.position.y = newPose.pose.position.y - offsets[1];
    newPose.pose.position.z = newPose.pose.position.z - offsets[2];

	// shift new pose orientation
	geometry_msgs::Quaternion quat_msg = msg->pose.pose.orientation;
	tf2::convert(quat_msg,quat_tf);
//	// Rotate current pose
	q_new = quat_offset_tf*quat_tf;  // Calculate the new orientation
	q_new.normalize();
	
	
	// Store rotated pose into new pose
	tf2::convert(q_new, newPose.pose.orientation);	

	newPose.header.frame_id = "world";

	// Previous Header
	path.poses.push_back(newPose);
	double roll1, pitch1, yaw1;
	tf2::convert(newPose.pose.orientation , quat_tf);
	tf2::Matrix3x3(quat_tf).getRPY(roll1, pitch1, yaw1);
	ROS_INFO("Yaw: [%f]", yaw1);
	path.header.seq = msg->header.seq;
	path.header.stamp = msg->header.stamp;
	path_pub.publish(path);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_convertor");

	ros::NodeHandle n;

	path_pub = n.advertise<nav_msgs::Path>("ground_truth_converted", 1000);
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("integrated_to_init", 1000,odomCallback);

	path.header.frame_id = "world";

	ros::Rate loop_rate(10);
	
	

	int count = 0;
	while (ros::ok())
	{
	
	path_pub.publish(path);

	// Take previous path and add odometry to current path


	ros::spinOnce();

	loop_rate.sleep();
	++count;
	}


  return 0;
}

///**
// * This file converts the odometry measurements from the dataset into a ground truth path
// */
//nav_msgs::Path path; 
//ros::Publisher path_pub;

//void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
//{
////	ROS_INFO("Seq: [%d]", msg->header.seq);
////	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
////	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
////	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
//	
//	// Variables
//	tf2::Quaternion q_orig, q_rot, q_new;
//	geometry_msgs::PoseStamped newPose;
//	geometry_msgs::Point translation; 
//	
//	// Get last pose
//	geometry_msgs::Pose lastPose = path.poses.back().pose;
//	
//	// Get the orientation of latest pose
//	tf2::convert(lastPose.orientation , q_orig);
//	
//	// Get the rotation of odometry
//	tf2::convert(msg->pose.pose.orientation , q_rot);

//	// Rotate current pose
//	q_new = q_rot*q_orig;  // Calculate the new orientation
//	q_new.normalize();

//	// Store rotated pose into new pose
//	tf2::convert(q_new, newPose.pose.orientation);	
//	
//	// Get the translation of latest pose
//	translation = msg->pose.pose.position;
//	
//	// Translate the latest pose into the newpose
//	newPose.pose.position.x = lastPose.position.x + translation.x;
//	newPose.pose.position.y = lastPose.position.y + translation.y;
//	newPose.pose.position.z = lastPose.position.z + translation.z;
//	
//	// Previous Header
//	path.poses.push_back(newPose);
//	path.header.seq++;
//	path.header.stamp = msg->header.stamp;
//	path_pub.publish(path);
//}



//int main(int argc, char **argv)
//{
//	ros::init(argc, argv, "odom_convertor");

//	ros::NodeHandle n;

//	path_pub = n.advertise<nav_msgs::Path>("ground_truth_converted", 1000);
//	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("integrated_to_init", 1000,odomCallback);

//	// Create initial path
//	//nav_msgs::Path path;

//	ros::Rate loop_rate(10);

//	int count = 0;
//	while (ros::ok())
//	{
//	

//	//    ROS_INFO("%s", msg.data.c_str());

//	path_pub.publish(path);


//	// Take previous path and add odometry to current path


//	ros::spinOnce();

//	loop_rate.sleep();
//	++count;
//	}


//  return 0;
//}

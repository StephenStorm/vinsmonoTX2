#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sstream>
#include <iostream>
#include <math.h> 

//#include <math>

using namespace std;

/**
 * This file converts the odometry measurements from the dataset into a ground truth path
 */
nav_msgs::Path path; 
ros::Publisher path_pub;
bool first_time = true;
int seq_offset;
geometry_msgs::PoseStamped newPose;
geometry_msgs::Quaternion quat_msg;
tf2::Quaternion quat,quat_rot,quat_current,quat_empty;
tf2::Transform trans_offset, trans_rot, trans_msg, trans_current;
tf2::Vector3 pt_current;

double roll, pitch, yaw;
double offset[4] = {0.0};

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if (first_time) 
	{
		first_time = false; 
		
  		// obtain the first yaw measurement
		quat_msg = msg->pose.pose.orientation;
 		tf2::convert(quat_msg ,quat);
		tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		quat_rot.setRPY(0,0,yaw);
		trans_rot.setRotation(quat_rot);
		
		// obtain the initial translation
		offset[0] = msg->pose.pose.position.x;
		offset[1] = msg->pose.pose.position.y;
		offset[2] = msg->pose.pose.position.z; 
		//pt_offset = Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
  		
	}
	
	// Subtract offsets from the point
	quat_empty.setRPY(0,0,0);
 	trans_current.setRotation(quat_empty);
	trans_current.setOrigin(tf2::Vector3(msg->pose.pose.position.x-offset[0], msg->pose.pose.position.y-offset[1], msg->pose.pose.position.z-offset[2]));
	
	// Then rotate the point
	trans_offset = trans_rot.inverseTimes(trans_current);
	// Convert to pose
	geometry_msgs::PoseStamped newPose;
	quat_current = trans_offset.getRotation();
	pt_current = trans_offset.getOrigin();
	tf2::convert(quat_current, newPose.pose.orientation);
	newPose.pose.position.x = pt_current.getX();
	newPose.pose.position.y = pt_current.getY();
	newPose.pose.position.z = pt_current.getZ(); 

    	// Update the header of th epose
    	newPose.header = msg->header;
	newPose.header.frame_id = "world";
	
	// Add the latest pose to the path
	path.poses.push_back(newPose);
	
	// Update the header of the path
	path.header.frame_id = "world";
	path.header.seq = msg->header.seq;
	path.header.stamp = msg->header.stamp;

	// Publish path
	path_pub.publish(path); 
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "tf_converter_node");

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

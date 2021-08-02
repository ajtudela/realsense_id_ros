/*
 * REALSENSE ID CLASS
 *
 * Copyright (c) 2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#include <ros/ros.h>
#include <realsense_id_ros/realsense_id_ros.h>

/* Main */
int main(int argc, char** argv){
	ros::init(argc, argv, "realsense_id");
	ros::NodeHandle node("");
	ros::NodeHandle node_private("~");

	try{
		ROS_INFO("[RealSense ID]: Initializing node");
		RealSenseIDROS realsense(node, node_private);
		while(ros::ok()){
			realsense.authenticateLoop();
			realsense.publishImage();
			ros::spinOnce();
		}
	}catch(const char* s){
		ROS_FATAL_STREAM("[RealSense ID]: " << s);
	}catch(...){
		ROS_FATAL_STREAM("[RealSense ID]: Unexpected error");
	}
}
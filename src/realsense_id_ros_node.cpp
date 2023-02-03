/*
 * REALSENSE ID CLASS
 *
 * Copyright (c) 2021-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "realsense_id_ros/realsense_id_ros.hpp"

/* Main */
int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<RealSenseIDROS>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
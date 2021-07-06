/*
 * REALSENSE ID ROS CLASS
 *
 * Copyright (c) 2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#ifndef REALSENSE_ID_H
#define REALSENSE_ID_H

// C++
#include <string>

// RealSense
#include <RealSenseID/FaceAuthenticator.h>
#include <RealSenseID/Logging.h>

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "realsense_callbacks.h"
#include "realsense_id_ros/Authenticate.h"
#include "realsense_id_ros/Enroll.h"
#include "realsense_id_ros/RemoveUser.h"
#include "realsense_id_ros/QueryUsersId.h"

class RealSenseIDROS{
	public:
		RealSenseIDROS(ros::NodeHandle& node, ros::NodeHandle& node_private);
		~RealSenseIDROS();

	private:
		ros::NodeHandle node_, nodePrivate_;
		ros::ServiceServer authSrv_, enrollSrv_, removeUserSrv_, removeAllSrv_, queryUsersIdSrv_;
		std::string port_;

		RSAuthenticationCallback authClbk_;
		RSEnrollmentCallback enrollClbk_;
		RealSenseID::FaceAuthenticator authenticator_;
		RealSenseID::SerialConfig config_;

		void getParams();
		bool authenticateService(realsense_id_ros::Authenticate::Request& req, realsense_id_ros::Authenticate::Response& res);
		bool enrollService(realsense_id_ros::Enroll::Request& req, realsense_id_ros::Enroll::Response& res);
		bool removeUserService(realsense_id_ros::RemoveUser::Request& req, realsense_id_ros::RemoveUser::Response& res);
		bool removeAllService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
		bool queryUsersIdService(realsense_id_ros::QueryUsersId::Request& req, realsense_id_ros::QueryUsersId::Response& res);
		void logCallback(RealSenseID::LogLevel level, const char* msg);
};

#endif

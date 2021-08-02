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

// OpenCV
#include <opencv2/core.hpp>

// RealSense
#include <RealSenseID/DeviceConfig.h>
#include <RealSenseID/FaceAuthenticator.h>
#include <RealSenseID/Logging.h>
#include <RealSenseID/Preview.h>

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>

#include "realsense_id_ros/Authenticate.h"
#include "realsense_id_ros/Enroll.h"
#include "realsense_id_ros/QueryUsersId.h"
#include "realsense_id_ros/RemoveUser.h"
#include "realsense_id_ros/RealSenseIDParametersConfig.h"
#include "realsense_id_ros/realsense_callbacks.h"

class RealSenseIDROS{
	public:
		RealSenseIDROS(ros::NodeHandle& node, ros::NodeHandle& node_private);
		~RealSenseIDROS();
		void publishImage();
		void authenticateLoop();

	private:
		ros::NodeHandle node_, nodePrivate_;
		ros::ServiceServer authSrv_, enrollSrv_, removeUserSrv_, removeAllSrv_, queryUsersIdSrv_;
		ros::Publisher imagePub_;
		dynamic_reconfigure::Server<realsense_id_ros::RealSenseIDParametersConfig> reconfigureSrv_;
		std::string port_;
		bool serverMode_, authLoopMode_;
		cv::Mat previewCVImage_;

		RealSenseID::FaceAuthenticator authenticator_;
		RealSenseID::SerialConfig serialConfig_;
		RealSenseID::DeviceConfig deviceConfig_;
		RealSenseID::PreviewConfig previewConfig_;
		RealSenseID::Preview preview_;
		RSPreviewCallback previewClbk_;

		void getParams();
		void logCallback(RealSenseID::LogLevel level, const char* msg);
		void reconfigureCallback(realsense_id_ros::RealSenseIDParametersConfig &config, uint32_t level);

		bool authenticateService(realsense_id_ros::Authenticate::Request& req, realsense_id_ros::Authenticate::Response& res);
		bool enrollService(realsense_id_ros::Enroll::Request& req, realsense_id_ros::Enroll::Response& res);
		bool removeUserService(realsense_id_ros::RemoveUser::Request& req, realsense_id_ros::RemoveUser::Response& res);
		bool removeAllService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
		bool queryUsersIdService(realsense_id_ros::QueryUsersId::Request& req, realsense_id_ros::QueryUsersId::Response& res);

		bool authenticateFaceprintsService(realsense_id_ros::Authenticate::Request& req, realsense_id_ros::Authenticate::Response& res);
		bool enrollFaceprintsService(realsense_id_ros::Enroll::Request& req, realsense_id_ros::Enroll::Response& res);
		bool removeUserFaceprintService(realsense_id_ros::RemoveUser::Request& req, realsense_id_ros::RemoveUser::Response& res);
		bool removeAllFaceprintsService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
		bool queryUsersIdFaceprintsService(realsense_id_ros::QueryUsersId::Request& req, realsense_id_ros::QueryUsersId::Response& res);
};

#endif

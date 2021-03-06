/*
 * REALSENSE ID ROS CLASS
 *
 * Copyright (c) 2021-2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
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
#include <thread>

// OpenCV
#include <opencv2/core.hpp>

// RealSense
#include <RealSenseID/DeviceController.h>
#include <RealSenseID/DeviceConfig.h>
#include <RealSenseID/FaceAuthenticator.h>
#include <RealSenseID/Logging.h>
#include <RealSenseID/Preview.h>
#include <RealSenseID/Version.h>

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include "realsense_id_ros/Authenticate.h"
#include "realsense_id_ros/DeviceInfo.h"
#include "realsense_id_ros/Enroll.h"
#include "realsense_id_ros/QueryUsersId.h"
#include "realsense_id_ros/RemoveUser.h"
#include "realsense_id_ros/RealSenseIDParametersConfig.h"
#include "realsense_id_ros/realsense_callbacks.h"
#include "realsense_id_ros/realsense_server_callbacks.h"
#include "realsense_id_ros/faceprintsDatabase.h"

class RealSenseIDROS{
	public:
		RealSenseIDROS(ros::NodeHandle& node, ros::NodeHandle& node_private);
		~RealSenseIDROS();
		void update();

	private:
		ros::NodeHandle node_, nodePrivate_;
		ros::Publisher facePub_, imagePub_, cameraInfoPub_;
		ros::ServiceServer getDevInfoSrv_, setCameraInfoSrv_, startAuthLoopSrv_, cancelAuthLoopSrv_;
		ros::ServiceServer authSrv_, enrollSrv_, removeUserSrv_, removeAllSrv_, queryUsersIdSrv_;
		dynamic_reconfigure::Server<realsense_id_ros::RealSenseIDParametersConfig> reconfigureSrv_;
		realsense_id_ros::RealSenseIDParametersConfig lastConfig_, defaultConfig_;
		sensor_msgs::CameraInfo cameraInfo_;
		std::thread authLoopThread_;
		bool setup_, running_;

		std::string port_, dbFilepath_, frameId_;
		bool serverMode_, authLoopMode_;
		cv::Mat previewCVImage_;

		RealSenseID::FaceAuthenticator authenticator_;
		RealSenseID::SerialConfig serialConfig_;
		RealSenseID::DeviceConfig deviceConfig_;
		RealSenseID::PreviewConfig previewConfig_;
		RSPreviewCallback previewClbk_;
		RSAuthenticationCallback authClbk_;
		RSAuthFaceprintsCallback authFaceClbk_;
		FaceprintsDatabase faceprintsDB_;
		std::unique_ptr<RealSenseID::Preview> preview_;

		void getParams();
		void logCallback(RealSenseID::LogLevel level, const char* msg);
		void reconfigureCallback(realsense_id_ros::RealSenseIDParametersConfig &config, uint32_t level);
		realsense_id_ros::Face detectionObjectToFace(std_msgs::Header header, DetectionObject detection, cv::Mat image);
		void authenticateLoop();

		bool getDeviceInfo(realsense_id_ros::DeviceInfo::Request& req, realsense_id_ros::DeviceInfo::Response& res);
		bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& res);
		bool startAuthenticationLoop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
		bool cancelAuthenticationLoop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

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

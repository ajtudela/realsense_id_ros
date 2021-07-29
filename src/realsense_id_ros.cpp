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

#include <chrono>
#include <thread>

#include <boost/bind.hpp>

// OpenCV
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include "realsense_id_ros/realsense_id_ros.h"

/* Initialize the subscribers, the publishers and the inference engine */
RealSenseIDROS::RealSenseIDROS(ros::NodeHandle& node, ros::NodeHandle& node_private): node_(node), nodePrivate_(node_private), preview_(previewConfig_){
	// Initialize ROS parameters
	getParams();

	// Initialize publishers and services
	authSrv_ = nodePrivate_.advertiseService("authenticate", &RealSenseIDROS::authenticateService, this);
	enrollSrv_ = nodePrivate_.advertiseService("enroll", &RealSenseIDROS::enrollService, this);
	removeUserSrv_ = nodePrivate_.advertiseService("remove_user", &RealSenseIDROS::removeUserService, this);
	removeAllSrv_ = nodePrivate_.advertiseService("remove_all", &RealSenseIDROS::removeAllService, this);
	queryUsersIdSrv_ = nodePrivate_.advertiseService("query_users_id", &RealSenseIDROS::queryUsersIdService, this);

	// Set log level
	RealSenseID::SetLogCallback(boost::bind(&RealSenseIDROS::logCallback, this, _1, _2), RealSenseID::LogLevel::Off, true);

	// Set serial config and connect
	serialConfig_.port = port_.c_str();
	auto status = authenticator_.Connect(serialConfig_);
	if(status != RealSenseID::Status::Ok){
		ROS_INFO_STREAM("[RealSense ID]: Failed connecting with status " << status);
	}else{
		ROS_INFO("[RealSense ID]: Connected to device.");
		ROS_DEBUG_STREAM("[RealSense ID]: Opening serial port " << port_.c_str());
	}

	// Set reconfigure srv
	reconfigureSrv_.setCallback(boost::bind(&RealSenseIDROS::reconfigureCallback, this, _1, _2));
}

/* Delete all parameteres */
RealSenseIDROS::~RealSenseIDROS(){
	// Disconnect from the device
	authenticator_.Disconnect();

	nodePrivate_.deleteParam("serial_port");
}

/* Update parameters of the node */
void RealSenseIDROS::getParams(){
	ROS_INFO("[RealSense ID]: Reading ROS parameters");

	nodePrivate_.param<std::string>("serial_port", port_, "/dev/ttyACM0");
}

/* Dynamic reconfigure callback */
void RealSenseIDROS::reconfigureCallback(realsense_id_ros::RealSenseIDParametersConfig &config, uint32_t level){
	ROS_INFO("[RealSense ID]: Reconfigure request for RealSenseID");

	// Change camera rotation
	if(config.camera_rotation == 0){
		deviceConfig_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_0_Deg;
	}else if(config.camera_rotation == 180){
		deviceConfig_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_180_Deg;
	}else if(config.camera_rotation == 90){
		deviceConfig_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_90_Deg;
	}else if(config.camera_rotation == 270){
		deviceConfig_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_270_Deg;
	}
	ROS_DEBUG_STREAM("[RealSense ID]: Camera rotation changed to " << config.camera_rotation);

	// Change security level
	if(config.security_level.find("high") != std::string::npos){
		deviceConfig_.security_level = RealSenseID::DeviceConfig::SecurityLevel::High;
	}else if(config.security_level.find("medium") != std::string::npos){
		deviceConfig_.security_level = RealSenseID::DeviceConfig::SecurityLevel::Medium;
	}else if(config.security_level.find("low") != std::string::npos){
		deviceConfig_.security_level = RealSenseID::DeviceConfig::SecurityLevel::Low;
	}
	ROS_DEBUG_STREAM("[RealSense ID]: Security level changed to " << config.security_level);

	// Change algorithm flow
	if(config.algo_flow.find("all") != std::string::npos){
		deviceConfig_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::All;
	}else if(config.algo_flow.find("detection") != std::string::npos){
		deviceConfig_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::FaceDetectionOnly;
	}else if(config.algo_flow.find("spoof") != std::string::npos){
		deviceConfig_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::SpoofOnly;
	}else if(config.algo_flow.find("recognition") != std::string::npos){
		deviceConfig_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::RecognitionOnly;
	}
	ROS_DEBUG_STREAM("[RealSense ID]: Algorithm flow changed to " << config.algo_flow);

	// Change face selection policy
	if(config.face_selection_policy.find("single") != std::string::npos){
		deviceConfig_.face_selection_policy = RealSenseID::DeviceConfig::FaceSelectionPolicy::Single;
	}else if(config.face_selection_policy.find("all") != std::string::npos){
		deviceConfig_.face_selection_policy = RealSenseID::DeviceConfig::FaceSelectionPolicy::All;
	}
	ROS_DEBUG_STREAM("[RealSense ID]: Face selection policy changed to " << config.face_selection_policy);

	// Change dump mode
	if(config.dump_mode.find("none") != std::string::npos){
		deviceConfig_.dump_mode = RealSenseID::DeviceConfig::DumpMode::None;
	}else if(config.dump_mode.find("cropped") != std::string::npos){
		deviceConfig_.dump_mode = RealSenseID::DeviceConfig::DumpMode::CroppedFace;
	}else if(config.dump_mode.find("fullframe") != std::string::npos){
		deviceConfig_.dump_mode = RealSenseID::DeviceConfig::DumpMode::FullFrame;
	}
	ROS_DEBUG_STREAM("[RealSense ID]: Dump mode changed to " << config.dump_mode);

	// Change matcher confidence level
	if(config.matcher_confidence_level.find("high") != std::string::npos){
		deviceConfig_.matcher_confidence_level = RealSenseID::DeviceConfig::MatcherConfidenceLevel::High;
	}else if(config.matcher_confidence_level.find("medium") != std::string::npos){
		deviceConfig_.matcher_confidence_level = RealSenseID::DeviceConfig::MatcherConfidenceLevel::Medium;
	}else if(config.matcher_confidence_level.find("low") != std::string::npos){
		deviceConfig_.matcher_confidence_level = RealSenseID::DeviceConfig::MatcherConfidenceLevel::Low;
	}
	ROS_DEBUG_STREAM("[RealSense ID]: Matcher confidence changed to " << config.matcher_confidence_level);

	auto status = authenticator_.SetDeviceConfig(deviceConfig_);
	if(status != RealSenseID::Status::Ok){
		ROS_ERROR("[RealSense ID]: Failed to apply device settings!");

		authenticator_.Disconnect();
	}
}

/* Perform one authentication */
bool RealSenseIDROS::authenticateService(realsense_id_ros::Authenticate::Request& req, realsense_id_ros::Authenticate::Response& res){
	ROS_INFO("[RealSense ID]: Authenticate service request");

	// Send config to callback in any case
	authClbk_.setDeviceConfig(deviceConfig_);

	// Clear all
	authClbk_.clear();

	// Authenticate a user
	auto status = authenticator_.Authenticate(authClbk_);

	// Start preview and wait until sensor is hot
	preview_.StartPreview(previewClbk_);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	if(status == RealSenseID::Status::Ok){
		std::vector<realsense_id_ros::Face> faces = authClbk_.newFaces;
		cv::Mat cvImage = previewClbk_.fullImage;

		// Continue if faces are detected
		if(faces.empty()) return false;

		// Rectangles for faces
		for(int r = 0; r < faces.size(); r++){
			realsense_id_ros::Rect rc = faces[r].face;
			std::string label = faces[r].label;

			cv::rectangle(cvImage, cv::Point2f(rc.x-1, rc.y), cv::Point2f(rc.x + 180, rc.y - 22), cv::Scalar(255, 0, 0.0), cv::FILLED, cv::LINE_AA);
			cv::putText(cvImage, label, cv::Point2f(rc.x, rc.y - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0), 1.5, cv::LINE_AA);
			cv::rectangle(cvImage, cv::Point2f(rc.x, rc.y), cv::Point2f(rc.x+rc.width, rc.y+rc.height), cv::Scalar(255, 0, 0), 4, cv::LINE_AA);
		}

		// Convert image to sensor_msgs
		cv_bridge::CvImage outputImageMsg;
		sensor_msgs::Image imageMsg;
		outputImageMsg.header.frame_id = "realsense_id_link";
		outputImageMsg.header.stamp = ros::Time::now();
		outputImageMsg.encoding = sensor_msgs::image_encodings::RGB8;
		outputImageMsg.image = cvImage;
		outputImageMsg.toImageMsg(imageMsg);

		res.faces = faces;
		res.source_img = imageMsg;
		preview_.StopPreview();

		return true;
	}

	return false;
}

/* Perform one enrollment for one new user */
bool RealSenseIDROS::enrollService(realsense_id_ros::Enroll::Request& req, realsense_id_ros::Enroll::Response& res){
	ROS_INFO("[RealSense ID]: Enroll service request");

	// Clear all
	enrollClbk_.clear();

	// Enroll a user
	auto status = authenticator_.Enroll(enrollClbk_, req.name.c_str());
	if(status == RealSenseID::Status::Ok){
		res.faces = enrollClbk_.newFaces;
		return true;
	}

	return false;
}

/* Attempt to remove specific user from the device. */
bool RealSenseIDROS::removeUserService(realsense_id_ros::RemoveUser::Request& req, realsense_id_ros::RemoveUser::Response& res){
	ROS_INFO("[RealSense ID]: Remove user service request");

	// Remove a user
	auto status = authenticator_.RemoveUser(req.name.c_str());
	if(status == RealSenseID::Status::Ok) return true;
	else return false;
}

/* Attempt to remove all users from the device. */
bool RealSenseIDROS::removeAllService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	ROS_INFO("[RealSense ID]: Remove all users service request");

	// Remove all users
	auto status = authenticator_.RemoveAll();
	if(status == RealSenseID::Status::Ok) return true;
	else return false;
}

/* Query the device about all enrolled users. */
bool RealSenseIDROS::queryUsersIdService(realsense_id_ros::QueryUsersId::Request& req, realsense_id_ros::QueryUsersId::Response& res){
	ROS_INFO("[RealSense ID]: Query users id service request");

	// Get number of users
	unsigned int numberOfUsers = 0;
	auto status = authenticator_.QueryNumberOfUsers(numberOfUsers);
	if(status != RealSenseID::Status::Ok){
		ROS_INFO_STREAM("[RealSense ID]: Status: " << status);
		return false;
	}

	if(numberOfUsers == 0){
		ROS_INFO("[RealSense ID]: No users found");
		return false;
	}

	// Allocate needed array of user ids
	char** userIds = new char*[numberOfUsers];
	for(unsigned i = 0; i < numberOfUsers; i++){
		userIds[i] = new char[RealSenseID::MAX_USERID_LENGTH];
	}
	unsigned int numberOfUsersInOut = numberOfUsers;
	status = authenticator_.QueryUserIds(userIds, numberOfUsersInOut);
	if(status != RealSenseID::Status::Ok){
		ROS_INFO_STREAM("[RealSense ID]: Status: " << status);
		// Free allocated memory and return on error
		for(unsigned int i = 0; i < numberOfUsers; i++){
			delete userIds[i];
		}
		delete[] userIds;
		return false;
	}

	// Create response
	res.number_users = numberOfUsersInOut;
	for(unsigned int i = 0; i < numberOfUsersInOut; i++){
		res.users_id.push_back(userIds[i]);
	}

	// Free allocated memory
	for(unsigned int i = 0; i < numberOfUsers; i++){
		delete userIds[i];
	}
	delete[] userIds;

	return true;
}

/* Logging callback */
void RealSenseIDROS::logCallback(RealSenseID::LogLevel level, const char* msg){
	ROS_INFO_STREAM("[RealSense ID]: " << msg);
}

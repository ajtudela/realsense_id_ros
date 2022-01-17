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

#include <chrono>
#include <thread>

#include <boost/bind.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>

// ROS
#include <vision_msgs/BoundingBox2D.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include "realsense_id_ros/Face.h"
#include "realsense_id_ros/realsense_id_ros.h"
#include "realsense_id_ros/realsense_server_callbacks.h"

/* Initialize the subscribers and the publishers */
RealSenseIDROS::RealSenseIDROS(ros::NodeHandle& node, ros::NodeHandle& node_private): node_(node), nodePrivate_(node_private),
																				preview_(previewConfig_), setup_(false){
	// Initialize ROS parameters
	getParams();

	// Initialize publishers and services
	if(!serverMode_){
		authSrv_ = nodePrivate_.advertiseService("authenticate", &RealSenseIDROS::authenticateService, this);
		enrollSrv_ = nodePrivate_.advertiseService("enroll", &RealSenseIDROS::enrollService, this);
		removeUserSrv_ = nodePrivate_.advertiseService("remove_user", &RealSenseIDROS::removeUserService, this);
		removeAllSrv_ = nodePrivate_.advertiseService("remove_all", &RealSenseIDROS::removeAllService, this);
		queryUsersIdSrv_ = nodePrivate_.advertiseService("query_users_id", &RealSenseIDROS::queryUsersIdService, this);
	}else{
		authSrv_ = nodePrivate_.advertiseService("authenticate", &RealSenseIDROS::authenticateFaceprintsService, this);
		enrollSrv_ = nodePrivate_.advertiseService("enroll", &RealSenseIDROS::enrollFaceprintsService, this);
		removeUserSrv_ = nodePrivate_.advertiseService("remove_user", &RealSenseIDROS::removeUserFaceprintService, this);
		removeAllSrv_ = nodePrivate_.advertiseService("remove_all", &RealSenseIDROS::removeAllFaceprintsService, this);
		queryUsersIdSrv_ = nodePrivate_.advertiseService("query_users_id", &RealSenseIDROS::queryUsersIdFaceprintsService, this);
	}

	imagePub_ = nodePrivate_.advertise<sensor_msgs::Image>("image_raw", 1);

	// Set log level
	RealSenseID::SetLogCallback(boost::bind(&RealSenseIDROS::logCallback, this, _1, _2), RealSenseID::LogLevel::Off, true);

	// Set serial config and connect
	serialConfig_.port = port_.c_str();
	auto status = authenticator_.Connect(serialConfig_);
	if(status != RealSenseID::Status::Ok){
		ROS_INFO_STREAM("[RealSense ID]: Failed connecting with status " << status);
	}else{
		ROS_INFO("[RealSense ID]: Connected to device");
		ROS_DEBUG_STREAM("[RealSense ID]: Opening serial port " << port_.c_str());
	}

	// Set reconfigure srv
	reconfigureSrv_.setCallback(boost::bind(&RealSenseIDROS::reconfigureCallback, this, _1, _2));

	// Start preview
	preview_.StartPreview(previewClbk_);
}

/* Delete all parameteres. */
RealSenseIDROS::~RealSenseIDROS(){
	// Disconnect from the device
	authenticator_.Disconnect();

	// Stop preview
	preview_.StopPreview();

	// Delete params
	nodePrivate_.deleteParam("serial_port");
	nodePrivate_.deleteParam("server_mode");
}

/* Update parameters of the node. */
void RealSenseIDROS::getParams(){
	ROS_INFO("[RealSense ID]: Reading ROS parameters");

	nodePrivate_.param<std::string>("serial_port", port_, "/dev/ttyACM0");
	nodePrivate_.param<bool>("server_mode", serverMode_, false);
}

/* Dynamic reconfigure callback. */
void RealSenseIDROS::reconfigureCallback(realsense_id_ros::RealSenseIDParametersConfig &config, uint32_t level){
	ROS_INFO("[RealSense ID]: Reconfigure request for RealSenseID");
	//The first time we're called, we just want to make sure we have the original configuration
	if(!setup_){
		lastConfig_ = config;
		defaultConfig_ = config;
		setup_ = true;
		return;
	}

	if(config.restore_defaults) {
		config = defaultConfig_;
		config.restore_defaults = false;
	}

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

	// Chaneg authenticate loop
	authLoopMode_ = config.authenticate_loop;
	ROS_DEBUG("[RealSense ID]: Authenticate loop changed");

	auto status = authenticator_.SetDeviceConfig(deviceConfig_);
	if(status != RealSenseID::Status::Ok){
		ROS_ERROR("[RealSense ID]: Failed to apply device settings!");

		authenticator_.Disconnect();
	}
}

/* Logging callback. */
void RealSenseIDROS::logCallback(RealSenseID::LogLevel level, const char* msg){
	ROS_INFO_STREAM("[RealSense ID]: " << msg);
}

/* Publish image */
void RealSenseIDROS::publishImage(){
	previewCVImage_ = previewClbk_.fullImage;
	// Convert to sensor msgs
	cv_bridge::CvImage cvImageBr;
	cvImageBr.header.frame_id = "realsense_id_link";
	cvImageBr.header.stamp = ros::Time::now();
	cvImageBr.encoding = sensor_msgs::image_encodings::RGB8;
	cvImageBr.image = previewCVImage_;
	imagePub_.publish(cvImageBr.toImageMsg());
}

/* Authenticate loop */
void RealSenseIDROS::authenticateLoop(){
	if(!authLoopMode_) return;

	ROS_INFO_ONCE("[RealSense ID]: Authenticate loop started");

	RSAuthenticationCallback authClbk;

	// Send config to callback
	authClbk.setDeviceConfig(deviceConfig_);

	// Authenticate a user
	auto status = authenticator_.Authenticate(authClbk);

	// Get timestamps saved in callbacks
	auto faceDetectionTs = authClbk.GetLastTimeStamp();

	if(status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = authClbk.GetDetections();

		// Create face array message
		std::vector<realsense_id_ros::Face> faces;
		for(DetectionObject detection: detections){

			// Create a rectangle with label
			cv::rectangle(previewCVImage_, cv::Point2f(detection.x-1, detection.y), cv::Point2f(detection.x + 180, detection.y - 22), cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_AA);
			cv::putText(previewCVImage_, detection.id, cv::Point2f(detection.x, detection.y - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0), 1.5, cv::LINE_AA);
			cv::rectangle(previewCVImage_, cv::Point2f(detection.x, detection.y), cv::Point2f(detection.x + detection.width, detection.y + detection.height), cv::Scalar(255, 0, 0), 4, cv::LINE_AA);
		}
	}
}

// -------------------- DEVICE MODE ---------------

/* Perform one authentication. */
bool RealSenseIDROS::authenticateService(realsense_id_ros::Authenticate::Request& req, realsense_id_ros::Authenticate::Response& res){
	ROS_INFO("[RealSense ID]: Authenticate service request");

	RSAuthenticationCallback authClbk;

	// Send config to callback
	authClbk.setDeviceConfig(deviceConfig_);

	// Authenticate a user
	auto status = authenticator_.Authenticate(authClbk);

	// Get timestamps saved in callbacks
	auto faceDetectionTs = authClbk.GetLastTimeStamp();

	if(status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = authClbk.GetDetections();

		// Exit if no faces are detected
		if(detections.empty()) return false;

		// Create face array message
		std::vector<realsense_id_ros::Face> faces;
		for(DetectionObject detection: detections){
			realsense_id_ros::Face face;

			// Header, id and confidence
			face.header.frame_id = "realsense_id_link";
			face.header.stamp = ros::Time::now();
			face.id = detection.id;
			face.confidence = detection.confidence;

			// 2D bounding box surrounding the object
			face.bbox.center.x = detection.x + detection.width / 2;
			face.bbox.center.y = detection.y + detection.height / 2;
			face.bbox.size_x = detection.width;
			face.bbox.size_y = detection.height;

			// The 2D data that generated these results
			cv::Mat croppedImage = previewCVImage_(cv::Rect(detection.x, detection.y, detection.width, detection.height));
			cv_bridge::CvImage cvImageBr;
			cvImageBr.header.frame_id = "realsense_id_link";
			cvImageBr.header.stamp = ros::Time::now();
			cvImageBr.encoding = sensor_msgs::image_encodings::RGB8;
			cvImageBr.image = croppedImage;
			cvImageBr.toImageMsg(face.source_img);

			faces.push_back(face);
		}

		res.faces = faces;
		return true;
	}

	return false;
}

/* Perform one enrollment for one new user. */
bool RealSenseIDROS::enrollService(realsense_id_ros::Enroll::Request& req, realsense_id_ros::Enroll::Response& res){
	ROS_INFO("[RealSense ID]: Enroll service request");

	RSEnrollmentCallback enrollClbk;

	// Enroll a user
	auto status = authenticator_.Enroll(enrollClbk, req.id.c_str());
	if(status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = enrollClbk.GetDetections();

		// Create face array message
		std::vector<realsense_id_ros::Face> faces;
		for(DetectionObject detection: detections){
			realsense_id_ros::Face face;

			// Header
			face.header.frame_id = "realsense_id_link";
			face.header.stamp = ros::Time::now();

			// 2D bounding box surrounding the object
			face.bbox.center.x = detection.x + detection.width / 2;
			face.bbox.center.y = detection.y + detection.height / 2;
			face.bbox.size_x = detection.width;
			face.bbox.size_y = detection.height;

			// The 2D data that generated these results
			cv::Mat croppedImage = previewCVImage_(cv::Rect(detection.x, detection.y, detection.width, detection.height));
			cv_bridge::CvImage cvImageBr;
			cvImageBr.header.frame_id = "realsense_id_link";
			cvImageBr.header.stamp = ros::Time::now();
			cvImageBr.encoding = sensor_msgs::image_encodings::RGB8;
			cvImageBr.image = croppedImage;
			cvImageBr.toImageMsg(face.source_img);

			faces.push_back(face);
		}

		res.faces = faces;
		return true;
	}

	return false;
}

/* Attempt to remove specific user from the device. */
bool RealSenseIDROS::removeUserService(realsense_id_ros::RemoveUser::Request& req, realsense_id_ros::RemoveUser::Response& res){
	ROS_INFO("[RealSense ID]: Remove user service request");

	// Remove a user
	auto status = authenticator_.RemoveUser(req.name.c_str());
	if(status == RealSenseID::Status::Ok){
		ROS_INFO("[RealSense ID]: User removed successfully");
		return true;
	}else{
		ROS_INFO("[RealSense ID]: User does not exists");
		return false;
	}
}

/* Attempt to remove all users from the device. */
bool RealSenseIDROS::removeAllService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	ROS_INFO("[RealSense ID]: Remove all users service request");

	// Remove all users
	auto status = authenticator_.RemoveAll();
	if(status == RealSenseID::Status::Ok){
		ROS_INFO("[RealSense ID]: Users removed successfully");
		return true;
	}else{
		ROS_INFO("[RealSense ID]: Database is already empty");
		return false;
	}
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

// -------------------- SERVER MODE ---------------

/*Perform one authentication in server mode. */
bool RealSenseIDROS::authenticateFaceprintsService(realsense_id_ros::Authenticate::Request& req, realsense_id_ros::Authenticate::Response& res){
	ROS_INFO("[RealSense ID]: Authenticate faceprints service request");

	// Extract faceprints of the user in front of the device
	RSAuthFaceprintsCallback authClbk(&authenticator_);
	auto status = authenticator_.ExtractFaceprintsForAuth(authClbk);
	if(status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = authClbk.GetDetections();

		// Exit if no faces are detected
		if(detections.empty()) return false;

		// Create face array message
		std::vector<realsense_id_ros::Face> faces;
		for(DetectionObject detection: detections){
			realsense_id_ros::Face face;

			// Header, id and confidence
			face.header.frame_id = "realsense_id_link";
			face.header.stamp = ros::Time::now();
			face.id = detection.id;
			face.confidence = detection.confidence;

			// 2D bounding box surrounding the object
			face.bbox.center.x = detection.x + detection.width / 2;
			face.bbox.center.y = detection.y + detection.height / 2;
			face.bbox.size_x = detection.width;
			face.bbox.size_y = detection.height;

			// The 2D data that generated these results
			cv::Mat croppedImage = previewCVImage_(cv::Rect(detection.x, detection.y, detection.width, detection.height));
			cv_bridge::CvImage cvImageBr;
			cvImageBr.header.frame_id = "realsense_id_link";
			cvImageBr.header.stamp = ros::Time::now();
			cvImageBr.encoding = sensor_msgs::image_encodings::RGB8;
			cvImageBr.image = croppedImage;
			cvImageBr.toImageMsg(face.source_img);

			faces.push_back(face);
		}

		res.faces = faces;
		return true;
	}

	return false;
}

/* Perform one enrollment for one new user in server mode. */
bool RealSenseIDROS::enrollFaceprintsService(realsense_id_ros::Enroll::Request& req, realsense_id_ros::Enroll::Response& res){
	ROS_INFO("[RealSense ID]: Enroll faceprints service request");

	// Enroll a user
	RSEnrollFaceprintsCallback enrollClbk{req.id.c_str()};
	auto status = authenticator_.ExtractFaceprintsForEnroll(enrollClbk);
	if(status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = enrollClbk.GetDetections();

		// Create face array message
		std::vector<realsense_id_ros::Face> faces;
		for(DetectionObject detection: detections){
			realsense_id_ros::Face face;

			// Header
			face.header.frame_id = "realsense_id_link";
			face.header.stamp = ros::Time::now();

			// 2D bounding box surrounding the object
			face.bbox.center.x = detection.x + detection.width / 2;
			face.bbox.center.y = detection.y + detection.height / 2;
			face.bbox.size_x = detection.width;
			face.bbox.size_y = detection.height;

			// The 2D data that generated these results
			cv::Mat croppedImage = previewCVImage_(cv::Rect(detection.x, detection.y, detection.width, detection.height));
			cv_bridge::CvImage cvImageBr;
			cvImageBr.header.frame_id = "realsense_id_link";
			cvImageBr.header.stamp = ros::Time::now();
			cvImageBr.encoding = sensor_msgs::image_encodings::RGB8;
			cvImageBr.image = croppedImage;

			faces.push_back(face);
		}

		res.faces = faces;
		return true;
	}

	return false;
}

/* Attempt to remove specific user from the database. */
bool RealSenseIDROS::removeUserFaceprintService(realsense_id_ros::RemoveUser::Request& req, realsense_id_ros::RemoveUser::Response& res){
	ROS_INFO("[RealSense ID]: Remove user faceprint service request");

	// Remove a user
	auto status = faceprintsDB.erase(req.name.c_str());
	if(status == 1){
		ROS_INFO("[RealSense ID]: User removed successfully");
		return true;
	}else{
		ROS_INFO("[RealSense ID]: User does not exists");
		return false;
	}
}

/* Attempt to remove all users from the database. */
bool RealSenseIDROS::removeAllFaceprintsService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	ROS_INFO("[RealSense ID]: Remove all users faceprints service request");

	// Delete database
	faceprintsDB.clear();
}

/* Query the device about all enrolled users in server mode. */
bool RealSenseIDROS::queryUsersIdFaceprintsService(realsense_id_ros::QueryUsersId::Request& req, realsense_id_ros::QueryUsersId::Response& res){
	ROS_INFO("[RealSense ID]: Query users id faceprints service request");

	// Create response
	res.number_users = faceprintsDB.size();
	for(const auto& iter: faceprintsDB){
		res.users_id.push_back(iter.first);
	}
	return true;
}

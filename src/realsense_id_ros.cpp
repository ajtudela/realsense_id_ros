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

// C++
#include <boost/bind.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>

// ROS
#include <vision_msgs/BoundingBox2D.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include "realsense_id_ros/FaceArray.h"
#include "realsense_id_ros/realsense_id_ros.h"
#include "realsense_id_ros/realsense_server_callbacks.h"

/* Initialize the subscribers and the publishers */
RealSenseIDROS::RealSenseIDROS(ros::NodeHandle& node, ros::NodeHandle& node_private): node_(node), nodePrivate_(node_private),
																	setup_(false), running_(false){
	// Initialize ROS parameters
	getParams();

	// Set log level
	RealSenseID::SetLogCallback(boost::bind(&RealSenseIDROS::logCallback, this, _1, _2), RealSenseID::LogLevel::Off, true);

	// Set serial config and connect
	serialConfig_.port = port_.c_str();
	auto connectStatus = authenticator_.Connect(serialConfig_);
	if(connectStatus != RealSenseID::Status::Ok){
		ROS_ERROR_STREAM("[RealSense ID]: Failed connecting to port " << serialConfig_.port << " status:" << connectStatus);
		exit(1);
	}else{
		ROS_INFO("[RealSense ID]: Connected to device");
		ROS_DEBUG_STREAM("[RealSense ID]: Opening serial port " << port_.c_str());
	}

	// Set reconfigure srv
	reconfigureSrv_.setCallback(boost::bind(&RealSenseIDROS::reconfigureCallback, this, _1, _2));

	// Initialize services
	getDevInfoSrv_ = nodePrivate_.advertiseService("device_info", &RealSenseIDROS::getDeviceInfo, this);
	setCameraInfoSrv_ = nodePrivate_.advertiseService("set_camera_info", &RealSenseIDROS::setCameraInfo, this);
	startAuthLoopSrv_ = nodePrivate_.advertiseService("start_authentication_loop", &RealSenseIDROS::startAuthenticationLoop, this);
	cancelAuthLoopSrv_ = nodePrivate_.advertiseService("cancel_authentication_loop", &RealSenseIDROS::cancelAuthenticationLoop, this);

	if(!serverMode_){
		ROS_INFO("[RealSense ID]: Using API in device mode");
		authSrv_ = nodePrivate_.advertiseService("authenticate", &RealSenseIDROS::authenticateService, this);
		enrollSrv_ = nodePrivate_.advertiseService("enroll", &RealSenseIDROS::enrollService, this);
		removeUserSrv_ = nodePrivate_.advertiseService("remove_user", &RealSenseIDROS::removeUserService, this);
		removeAllSrv_ = nodePrivate_.advertiseService("remove_all", &RealSenseIDROS::removeAllService, this);
		queryUsersIdSrv_ = nodePrivate_.advertiseService("query_users_id", &RealSenseIDROS::queryUsersIdService, this);
	}else{
		ROS_INFO("[RealSense ID]: Using API in server mode");
		authSrv_ = nodePrivate_.advertiseService("authenticate", &RealSenseIDROS::authenticateFaceprintsService, this);
		enrollSrv_ = nodePrivate_.advertiseService("enroll", &RealSenseIDROS::enrollFaceprintsService, this);
		removeUserSrv_ = nodePrivate_.advertiseService("remove_user", &RealSenseIDROS::removeUserFaceprintService, this);
		removeAllSrv_ = nodePrivate_.advertiseService("remove_all", &RealSenseIDROS::removeAllFaceprintsService, this);
		queryUsersIdSrv_ = nodePrivate_.advertiseService("query_users_id", &RealSenseIDROS::queryUsersIdFaceprintsService, this);
	}

	// And publishers
	facePub_ = nodePrivate_.advertise<realsense_id_ros::FaceArray>("faces", 1);
	imagePub_ = nodePrivate_.advertise<sensor_msgs::Image>("image_raw", 10);
	cameraInfoPub_ = nodePrivate_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

	// Load the database from a file
	if(serverMode_){
		if(!dbFilepath_.empty()){
			faceprintsDB_.loadDbFromFile(dbFilepath_);
			ROS_INFO("[RealSense ID]: Faceprints database load from file");
		}
	}

	// Change authenticate loop
	std_srvs::Empty empt;
	if(!running_ && authLoopMode_){
		startAuthenticationLoop(empt.request, empt.response);
	}
}

/* Delete all parameteres. */
RealSenseIDROS::~RealSenseIDROS(){
	// Cancel
	authenticator_.Cancel();

	// Disconnect from the device
	authenticator_.Disconnect();

	// Stop preview
	preview_->StopPreview();

	// Delete params
	nodePrivate_.deleteParam("serial_port");
	nodePrivate_.deleteParam("authenticate_loop");
	nodePrivate_.deleteParam("server_mode");
	nodePrivate_.deleteParam("database");
}

/* Update parameters of the node. */
void RealSenseIDROS::getParams(){
	ROS_INFO("[RealSense ID]: Reading ROS parameters");

	frameId_ = "realsense_id_optical_frame";

	nodePrivate_.param<std::string>("serial_port", port_, "/dev/ttyACM0");
	nodePrivate_.param<bool>("authenticate_loop", authLoopMode_, false);
	nodePrivate_.param<bool>("server_mode", serverMode_, false);
	nodePrivate_.param<std::string>("database", dbFilepath_, "");
}

/* Dynamic reconfigure callback. */
void RealSenseIDROS::reconfigureCallback(realsense_id_ros::RealSenseIDParametersConfig &config, uint32_t level){
	ROS_INFO("[RealSense ID]: Reconfigure request for RealSenseID");
	//The first time we're called, we just want to make sure we have the original configuration
	if(!setup_){
		lastConfig_ = config;
		defaultConfig_ = config;
		setup_ = true;
		auto status = authenticator_.QueryDeviceConfig(deviceConfig_);
		ROS_INFO("[RealSense ID]: Authentication settings:");
		ROS_INFO_STREAM("[RealSense ID]: * Rotation: " << deviceConfig_.camera_rotation);
		ROS_INFO_STREAM("[RealSense ID]: * Security: " << deviceConfig_.security_level);
		ROS_INFO_STREAM("[RealSense ID]: * Algo flow Mode: " << deviceConfig_.algo_flow);
		ROS_INFO_STREAM("[RealSense ID]: * Face policy: " << deviceConfig_.face_selection_policy);
		ROS_INFO_STREAM("[RealSense ID]: * Dump Mode: " << deviceConfig_.dump_mode);
		ROS_INFO_STREAM("[RealSense ID]: * Matcher Confidence Level: " << deviceConfig_.matcher_confidence_level);
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

/* Create a Face msgs */
realsense_id_ros::Face RealSenseIDROS::detectionObjectToFace(std_msgs::Header header, DetectionObject detection, cv::Mat image){
	realsense_id_ros::Face face;

	// Header, id and confidence
	face.header = header;
	face.id = detection.id;
	face.confidence = detection.confidence;
	face.has_mask = detection.hasMask;

	// 2D bounding box surrounding the object
	face.bbox.center.x = detection.x + detection.width / 2;
	face.bbox.center.y = detection.y + detection.height / 2;
	face.bbox.size_x = detection.width;
	face.bbox.size_y = detection.height;

	// The 2D data that generated these results
	if(!image.empty()){
		cv::Mat croppedImage = image(cv::Rect(detection.x, detection.y, detection.width, detection.height));
		cv_bridge::CvImage cvImageBr;
		cvImageBr.header.frame_id = frameId_;
		cvImageBr.header.stamp = ros::Time::now();
		cvImageBr.encoding = sensor_msgs::image_encodings::RGB8;
		cvImageBr.image = croppedImage;
		cvImageBr.toImageMsg(face.source_img);
	}

	return face;
}

/* Authenticate loop */
void RealSenseIDROS::authenticateLoop(){
	if(!serverMode_){
		auto status = authenticator_.AuthenticateLoop(authClbk_);
	}else{
		authFaceClbk_.setAuthenticator(&authenticator_);
		authFaceClbk_.setFaceprintsDatabase(faceprintsDB_.data);
		auto status = authenticator_.ExtractFaceprintsForAuthLoop(authFaceClbk_);
	}
}

/* Publish Faces */
void RealSenseIDROS::update(){
	if(!authLoopMode_) return;

	// Create header of FaceArray
	realsense_id_ros::FaceArray faceArray;
	faceArray.header.frame_id = frameId_;
	faceArray.header.stamp = ros::Time::now();

	// Get image
	previewCVImage_ = previewClbk_.fullImage;
	const size_t colorHeight = (size_t) previewCVImage_.size().height;
	const size_t colorWidth  = (size_t) previewCVImage_.size().width;

	// Create face array message
	std::vector<DetectionObject> detections;
	if(!serverMode_){
		detections = authClbk_.GetDetections();
	}else{
		detections = authFaceClbk_.GetDetections();
	}

	for(DetectionObject &detection: detections){
		realsense_id_ros::Face face = detectionObjectToFace(faceArray.header, detection, previewClbk_.fullImage);
		faceArray.faces.push_back(face);

		// Improve bounding box
		detection.x = detection.x < 0 ? 0 : detection.x;
		detection.y = detection.y < 0 ? 0 : detection.y;
		detection.width = detection.width > colorWidth ? colorWidth : detection.width;
		detection.height = detection.height > colorHeight ? colorHeight : detection.height;
		// Color of the person
		cv::Scalar color;
		if(detection.id == "Spoof" || detection.id.empty()) color = cv::Scalar(255, 0, 0);
		else color = cv::Scalar(0, 255, 0);
		// Text label
		std::ostringstream conf;
		conf << ":" << std::fixed << std::setprecision(3) << detection.confidence;
		std::string labelText = detection.id + conf.str();
		// Rectangles for faces
		cv::rectangle(previewCVImage_, cv::Point2f(detection.x-1, detection.y), cv::Point2f(detection.x + 300, detection.y - 40), color, cv::FILLED, cv::LINE_AA);
		cv::putText(previewCVImage_, labelText, cv::Point2f(detection.x, detection.y - 5), cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 0), 1.5, cv::LINE_AA);
		cv::rectangle(previewCVImage_, cv::Point2f(detection.x, detection.y), cv::Point2f(detection.x + detection.width, detection.y + detection.height), color, 4, cv::LINE_AA);
	}

	// Publish array of faces
	facePub_.publish(faceArray);

	// Clear
	if(!serverMode_){
		authClbk_.clear();
	}else{
		authFaceClbk_.clear();
	}

	// Convert CV image to sensor msgs
	cv_bridge::CvImage cvImageBr;
	cvImageBr.header = faceArray.header;
	cvImageBr.encoding = sensor_msgs::image_encodings::RGB8;
	cvImageBr.image = previewCVImage_;
	imagePub_.publish(cvImageBr.toImageMsg());

	//Publish camera info
	cameraInfo_.header = faceArray.header;
	cameraInfo_.height = colorHeight;
	cameraInfo_.width = colorWidth;
	cameraInfo_.distortion_model = "plumb_bob";
	cameraInfo_.D = {0.0, 0.0, 0.0, 0.0, 0.0};
	cameraInfo_.K = {911.9729056029453, 0.0, 543.4705406497254, 0.0, 935.5803580030122, 902.0450795440844, 0.0, 0.0, 1.0};
	cameraInfo_.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	cameraInfo_.P = {999.5663452148438, 0.0, 557.4771347235219, 0.0, 0.0, 980.9320678710938, 835.6312784614311, 0.0, 0.0, 0.0, 1.0, 0.0};
	cameraInfoPub_.publish(cameraInfo_);
}

/* Start authentication loop */
bool RealSenseIDROS::startAuthenticationLoop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	ROS_INFO("[RealSense ID]: Start authentication loop service request");

	if(!running_){
		preview_ = std::make_unique<RealSenseID::Preview>(previewConfig_);
		preview_->StartPreview(previewClbk_);
		authLoopThread_ = std::thread(std::bind(&RealSenseIDROS::authenticateLoop, this));
		running_ = true;
		authLoopMode_ = true;
		nodePrivate_.setParam("authenticate_loop", authLoopMode_);
		ROS_INFO_ONCE("[RealSense ID]: Authentication loop started");
		return true;
	}else{
		ROS_ERROR("[RealSense ID]: Cannot start the authentication loop");
		return false;
	}
}

/* Cancel authentication loop */
bool RealSenseIDROS::cancelAuthenticationLoop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	ROS_INFO("[RealSense ID]: Cancel authentication loop service request");

	if(running_){
		authenticator_.Cancel();
		preview_->StopPreview();
		authLoopThread_.join();
		running_ = false;
		authLoopMode_ = false;
		nodePrivate_.setParam("authenticate_loop", authLoopMode_);
		ROS_INFO("[RealSense ID]: Authentication loop stopped");
		return true;
	}else{
		nodePrivate_.setParam("authenticate_loop", authLoopMode_);
		ROS_ERROR("[RealSense ID]: Cannot cancel the authentication loop");
		return false;
	}
}

/* Get device info. */
bool RealSenseIDROS::getDeviceInfo(realsense_id_ros::DeviceInfo::Request& req, realsense_id_ros::DeviceInfo::Response& res){
	ROS_INFO("[RealSense ID]: Get device info service request");

	RealSenseID::DeviceController deviceController;

	auto connectStatus = deviceController.Connect(serialConfig_);
	if(connectStatus != RealSenseID::Status::Ok){
		ROS_INFO_STREAM("[RealSense ID]: Failed connecting to port " << serialConfig_.port << " status:" << connectStatus);
		return false;
	}

	std::string firmwareVersion;
	auto status = deviceController.QueryFirmwareVersion(firmwareVersion);
	if(status != RealSenseID::Status::Ok){
		ROS_INFO("[RealSense ID]: Failed getting firmware version!");
		return false;
	}

	std::string serialNumber;
	status = deviceController.QuerySerialNumber(serialNumber);
	if(status != RealSenseID::Status::Ok){
		ROS_INFO("[RealSense ID]: Failed getting serial number!");
		return false;
	}

	deviceController.Disconnect();

	std::string hostVersion = RealSenseID::Version();

	res.device_name = "Intel Realsense F450 / F455";
	res.serial_number = serialNumber;
	res.firmware_version = firmwareVersion;
	res.host_version = hostVersion;

	return true;
}

/* Set camera info */
bool RealSenseIDROS::setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& res){
	ROS_INFO("[RealSense ID]: Set camera info service request");

	cameraInfo_ = req.camera_info;

	if(!node_.ok()){
		ROS_ERROR("[RealSense ID]: Camera driver not running.");
		res.status_message = "Camera driver not running.";
		res.success = false;
		return false;
	}

	return true;
}

// -------------------- DEVICE MODE ---------------

/* Perform one authentication. */
bool RealSenseIDROS::authenticateService(realsense_id_ros::Authenticate::Request& req, realsense_id_ros::Authenticate::Response& res){
	ROS_INFO("[RealSense ID]: Authenticate service request");

	bool success = false;

	// Create header of FaceArray
	realsense_id_ros::FaceArray faceArray;
	faceArray.header.frame_id = frameId_;
	faceArray.header.stamp = ros::Time::now();

	// Start preview
	preview_ = std::make_unique<RealSenseID::Preview>(previewConfig_);
	preview_->StartPreview(previewClbk_);

	// Authenticate a user
	RSAuthenticationCallback authClbk;
	auto status = authenticator_.Authenticate(authClbk);

	// Get timestamps saved in callbacks
	auto faceDetectionTs = authClbk.GetLastTimeStamp();

	// Detections ...
	if(status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = authClbk.GetDetections();

		// Exit if no faces are detected
		if(detections.empty()){
			preview_->StopPreview();
			return false;
		}

		// Create face array message
		std::vector<realsense_id_ros::Face> faces;
		for(const DetectionObject &detection: detections){
			realsense_id_ros::Face face = detectionObjectToFace(faceArray.header, detection, previewClbk_.fullImage);
			faces.push_back(face);
		}

		res.faces = faces;
		success = true;
	}

	// Stop preview
	preview_->StopPreview();

	return success;
}

/* Perform one enrollment for one new user. */
bool RealSenseIDROS::enrollService(realsense_id_ros::Enroll::Request& req, realsense_id_ros::Enroll::Response& res){
	ROS_INFO("[RealSense ID]: Enroll service request");

	bool success = false;

	// Create header of FaceArray
	realsense_id_ros::FaceArray faceArray;
	faceArray.header.frame_id = frameId_;
	faceArray.header.stamp = ros::Time::now();

	// Start preview
	preview_ = std::make_unique<RealSenseID::Preview>(previewConfig_);
	preview_->StartPreview(previewClbk_);

	// Enroll a user
	RSEnrollmentCallback enrollClbk;
	auto status = authenticator_.Enroll(enrollClbk, req.id.c_str());

	// Detections ...
	if(status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = enrollClbk.GetDetections();

		// Create face array message
		std::vector<realsense_id_ros::Face> faces;
		for(DetectionObject detection: detections){
			detection.id = req.id.c_str();
			realsense_id_ros::Face face = detectionObjectToFace(faceArray.header, detection, previewClbk_.fullImage);
			faces.push_back(face);
		}

		res.faces = faces;
		success = true;
	}

	// Stop preview
	preview_->StopPreview();

	return success;
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

/* Perform one authentication in server mode. */
bool RealSenseIDROS::authenticateFaceprintsService(realsense_id_ros::Authenticate::Request& req, realsense_id_ros::Authenticate::Response& res){
	ROS_INFO("[RealSense ID]: Authenticate faceprints service request");

	bool success = false;

	// Create header of FaceArray
	realsense_id_ros::FaceArray faceArray;
	faceArray.header.frame_id = frameId_;
	faceArray.header.stamp = ros::Time::now();

	// Start preview
	preview_ = std::make_unique<RealSenseID::Preview>(previewConfig_);
	preview_->StartPreview(previewClbk_);

	// Create callback
	RSAuthFaceprintsCallback authClbk(&authenticator_, faceprintsDB_.data);

	// Extract faceprints of the user in front of the device
	auto status = authenticator_.ExtractFaceprintsForAuth(authClbk);
	if(status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = authClbk.GetDetections();

		// Exit if no faces are detected
		if(detections.empty()){
			preview_->StopPreview();
			return false;
		}

		// Create face array message
		std::vector<realsense_id_ros::Face> faces;
		for(const DetectionObject &detection: detections){
			realsense_id_ros::Face face = detectionObjectToFace(faceArray.header, detection, previewClbk_.fullImage);
			faces.push_back(face);
		}

		res.faces = faces;
		success = true;
	}

	// Stop preview
	preview_->StopPreview();

	return success;
}

/* Perform one enrollment for one new user in server mode. */
bool RealSenseIDROS::enrollFaceprintsService(realsense_id_ros::Enroll::Request& req, realsense_id_ros::Enroll::Response& res){
	ROS_INFO("[RealSense ID]: Enroll faceprints service request");

	bool success = false;

	// Start preview
	preview_ = std::make_unique<RealSenseID::Preview>(previewConfig_);
	preview_->StartPreview(previewClbk_);

	// Create header of FaceArray
	realsense_id_ros::FaceArray faceArray;
	faceArray.header.frame_id = frameId_;
	faceArray.header.stamp = ros::Time::now();

	// Create callback
	RSEnrollFaceprintsCallback enrollClbk(req.id.c_str(), faceprintsDB_.data);

	// Enroll a user
	auto status = authenticator_.ExtractFaceprintsForEnroll(enrollClbk);
	if(status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = enrollClbk.GetDetections();

		// Create face array message
		std::vector<realsense_id_ros::Face> faces;
		for(DetectionObject detection: detections){
			detection.id = req.id.c_str();
			realsense_id_ros::Face face = detectionObjectToFace(faceArray.header, detection, previewClbk_.fullImage);
			faces.push_back(face);
		}

		res.faces = faces;
		success = true;
	}

	// Stop preview
	preview_->StopPreview();

	// Save database to file
	faceprintsDB_.data = enrollClbk.getDatabase();

	return success;
}

/* Attempt to remove specific user from the database. */
bool RealSenseIDROS::removeUserFaceprintService(realsense_id_ros::RemoveUser::Request& req, realsense_id_ros::RemoveUser::Response& res){
	ROS_INFO("[RealSense ID]: Remove user faceprint service request");

	// Remove a user
	auto status = faceprintsDB_.removeUser(req.name.c_str());
	if(status){
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
	faceprintsDB_.removeAllUsers();
}

/* Query the device about all enrolled users in server mode. */
bool RealSenseIDROS::queryUsersIdFaceprintsService(realsense_id_ros::QueryUsersId::Request& req, realsense_id_ros::QueryUsersId::Response& res){
	ROS_INFO("[RealSense ID]: Query users id faceprints service request");

	// Create response
	res.number_users = faceprintsDB_.getNumberOfUsers();
	res.users_id = faceprintsDB_.getUsers();
	return true;
}

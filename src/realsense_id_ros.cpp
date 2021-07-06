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
#include <boost/bind.hpp>
#include "realsense_id_ros/realsense_id_ros.h"

/* Initialize the subscribers, the publishers and the inference engine */
RealSenseIDROS::RealSenseIDROS(ros::NodeHandle& node, ros::NodeHandle& node_private): node_(node), nodePrivate_(node_private){
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

	// Set config and connect
	config_.port = port_.c_str();
	auto status = authenticator_.Connect(config_);
	if(status != RealSenseID::Status::Ok){
		ROS_INFO_STREAM("[RealSense ID]: Failed connecting with status " << status);
	}
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

/* Perform one authentication */
bool RealSenseIDROS::authenticateService(realsense_id_ros::Authenticate::Request& req, realsense_id_ros::Authenticate::Response& res){
	ROS_INFO("[RealSense ID]: Authenticate service request");

	// Authenticate a user
	auto status = authenticator_.Authenticate(authClbk_);
	if(status == RealSenseID::Status::Ok){
		res.faces = authClbk_.newFaces;
		return true;
	}

	return false;
}

/* Perform one enrollment for one new user */
bool RealSenseIDROS::enrollService(realsense_id_ros::Enroll::Request& req, realsense_id_ros::Enroll::Response& res){
	ROS_INFO("[RealSense ID]: Enroll service request");

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
		userIds[i] = new char[RealSenseID::FaceAuthenticator::MAX_USERID_LENGTH];
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

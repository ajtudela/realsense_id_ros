/*
 * REALSENSE CALLBACKS CLASSES
 *
 * Copyright (c) 2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#ifndef REALSENSE_CALLBACKS_H
#define REALSENSE_CALLBACKS_H

// C++
#include <string>

// RealSense ID
#include <RealSenseID/AuthenticationCallback.h>
#include <RealSenseID/AuthenticateStatus.h>
#include <RealSenseID/EnrollmentCallback.h>
#include <RealSenseID/EnrollStatus.h>
#include <RealSenseID/FacePose.h>
#include <RealSenseID/FaceRect.h>

// ROS
#include <ros/ros.h>

#include "realsense_id_ros/Face.h"

class RSAuthenticationCallback: public RealSenseID::AuthenticationCallback{
	public:
		std::string userId;
		std::vector<realsense_id_ros::Face> newFaces;

		void OnResult(const RealSenseID::AuthenticateStatus status, const char* userId) override{
			if(status == RealSenseID::AuthenticateStatus::Success){
				this->userId = userId;
				ROS_INFO_STREAM("[RealSense ID]: Authenticated, userdId = " << userId);
			}else if (status == RealSenseID::AuthenticateStatus::Forbidden){
				ROS_INFO("[RealSense ID]: User is not authenticated");
			}else if (status == RealSenseID::AuthenticateStatus::Spoof){
				ROS_INFO("[RealSense ID]: Spoof");
			}else{
				ROS_INFO_STREAM("[RealSense ID]: " << status);
			}

			// Clear all detected faces
			newFaces.clear();

			for(auto& face: faces_){
				// Create Face msg
				realsense_id_ros::Face newFace;
				newFace.face.x = face.x;
				newFace.face.y = face.y;
				newFace.face.width = face.w;
				newFace.face.height = face.h;
				newFace.label = userId;

				ROS_INFO("[RealSense ID]: Detected face %u,%u %ux%u", face.x, face.y, face.w, face.h);

				newFaces.push_back(newFace);
			}
		}

		void OnHint(const RealSenseID::AuthenticateStatus hint) override{
			ROS_INFO_STREAM("[RealSense ID]: OnHint " << hint);
		}

		void OnFaceDetected(const std::vector<RealSenseID::FaceRect>& faces, const unsigned int ts) override{
			faces_ = faces;
		}
	private:
		std::vector<RealSenseID::FaceRect> faces_;
};

class RSEnrollmentCallback: public RealSenseID::EnrollmentCallback{
	public:
		std::vector<realsense_id_ros::Face> newFaces;

		void OnResult(const RealSenseID::EnrollStatus status) override{
			ROS_INFO_STREAM("[RealSense ID]: Result " << status);
		}

		void OnProgress(const RealSenseID::FacePose pose) override{
			ROS_INFO_STREAM("[RealSense ID]: OnProgress " << pose);
		}

		void OnHint(const RealSenseID::EnrollStatus hint) override{
			ROS_INFO_STREAM("[RealSense ID]: Hint " << hint);
		}

		void OnFaceDetected(const std::vector<RealSenseID::FaceRect>& faces, const unsigned int ts) override{
			// Clear all detected faces
			newFaces.clear();

			for(auto& face: faces){
				// Create Face msg
				realsense_id_ros::Face newFace;
				newFace.face.x = face.x;
				newFace.face.y = face.y;
				newFace.face.width = face.w;
				newFace.face.height = face.h;

				ROS_INFO("[RealSense ID]: Detected face %u,%u %ux%u", face.x, face.y, face.w, face.h);

				newFaces.push_back(newFace);
			}
		}
};

#endif

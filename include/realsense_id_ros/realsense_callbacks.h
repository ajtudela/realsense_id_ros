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

// OpenCV
#include <opencv2/core.hpp>

// RealSense ID
#include <RealSenseID/AuthenticationCallback.h>
#include <RealSenseID/AuthenticateStatus.h>
#include <RealSenseID/EnrollmentCallback.h>
#include <RealSenseID/EnrollStatus.h>
#include <RealSenseID/DeviceConfig.h>
#include <RealSenseID/FacePose.h>
#include <RealSenseID/FaceRect.h>
#include <RealSenseID/Preview.h>

// ROS
#include <ros/ros.h>

#include "detectionObject.h"

class RSAuthenticationCallback: public RealSenseID::AuthenticationCallback{
	public:
		void clear(){
			detections_.clear();
			results_ = 0;
		};

		void setDeviceConfig(RealSenseID::DeviceConfig devConfig){deviceConfig_ = devConfig;};

		void OnResult(const RealSenseID::AuthenticateStatus status, const char* userId) override{
			bool spoof = false;

			if(status == RealSenseID::AuthenticateStatus::Success){
				if(deviceConfig_.algo_flow == RealSenseID::DeviceConfig::AlgoFlow::SpoofOnly){
					ROS_INFO("[RealSense ID]: Real face");
				}else{
					ROS_INFO_STREAM("[RealSense ID]: Authenticated, userdId = " << userId);
				}
			}else if (status == RealSenseID::AuthenticateStatus::Forbidden){
				ROS_INFO("[RealSense ID]: User is not authenticated");
			}else if (status == RealSenseID::AuthenticateStatus::Spoof){
				spoof = true;
				ROS_INFO("[RealSense ID]: Spoof");
			}else if (status == RealSenseID::AuthenticateStatus::NoFaceDetected){
				ROS_DEBUG("[RealSense ID]: NoFaceDetected");
			}else{
				ROS_DEBUG_STREAM("[RealSense ID]: Authenticate " << status);
			}

			// Check results and add them to objects
			if(faces_.size() > results_){
				auto& face = faces_[results_];

				// Create new detection
				DetectionObject newDetection;
				newDetection.x = face.x;
				newDetection.y = face.y;
				newDetection.width = face.w;
				newDetection.height = face.h;
				newDetection.hasMask = false;
				newDetection.id = spoof ? "Spoof": userId;
				newDetection.confidence = -1;
				detections_.push_back(newDetection);

				ROS_DEBUG("[RealSense ID]: Detected face %u,%u %ux%u", face.x, face.y, face.w, face.h);
			}
			results_++;
		}

		void OnHint(const RealSenseID::AuthenticateStatus hint) override{
			ROS_DEBUG_STREAM("[RealSense ID]: Hint " << hint);
		}

		void OnFaceDetected(const std::vector<RealSenseID::FaceRect>& faces, const unsigned int ts) override{
			faces_ = faces;
			ts_ = ts;
		}

		unsigned int GetLastTimeStamp(){
			return ts_;
		}

		const std::vector<DetectionObject>& GetDetections(){
			return detections_;
		}

	private:
		std::vector<DetectionObject> detections_;
		std::vector<RealSenseID::FaceRect> faces_;
		RealSenseID::DeviceConfig deviceConfig_;
		size_t results_ = 0;
		unsigned int ts_ = 0;
};

class RSEnrollmentCallback: public RealSenseID::EnrollmentCallback{
	public:
		void OnResult(const RealSenseID::EnrollStatus status) override{
			if(status == RealSenseID::EnrollStatus::Success){
				ROS_INFO("[RealSense ID]: Real face");
			}else if (status == RealSenseID::EnrollStatus::Spoof){
				ROS_INFO("[RealSense ID]: Spoof");
			}else if (status == RealSenseID::EnrollStatus::NoFaceDetected){
				ROS_INFO("[RealSense ID]: NoFaceDetected");
			}else{
				ROS_DEBUG_STREAM("[RealSense ID]: Result " << status);
			}

			// Check results and add them to objects
			if(faces_.size() > results_){
				auto& face = faces_[results_];

				// Create new detection
				DetectionObject newDetection;
				newDetection.x = face.x;
				newDetection.y = face.y;
				newDetection.width = face.w;
				newDetection.height = face.h;
				newDetection.confidence = 1.0;
				newDetection.hasMask = false;
				detections_.push_back(newDetection);

				ROS_DEBUG("[RealSense ID]: Detected face %u,%u %ux%u", face.x, face.y, face.w, face.h);
			}
			results_++;
		}

		void OnProgress(const RealSenseID::FacePose pose) override{
			ROS_DEBUG_STREAM("[RealSense ID]: Progress " << pose);
		}

		void OnHint(const RealSenseID::EnrollStatus hint) override{
			ROS_DEBUG_STREAM("[RealSense ID]: Hint " << hint);
		}

		void OnFaceDetected(const std::vector<RealSenseID::FaceRect>& faces, const unsigned int ts) override{
			faces_ = faces;
			ts_ = ts;
		}

		unsigned int GetLastTimeStamp(){
			return ts_;
		}

		const std::vector<DetectionObject>& GetDetections(){
			return detections_;
		}

	private:
		std::vector<DetectionObject> detections_;
		std::vector<RealSenseID::FaceRect> faces_;
		size_t results_ = 0;
		unsigned int ts_ = 0;
};

class RSPreviewCallback: public RealSenseID::PreviewImageReadyCallback{
	public:
		cv::Mat fullImage;

		void OnPreviewImageReady(const RealSenseID::Image image){
			// Convert to CV Mat
			fullImage = cv::Mat(image.height, image.width, CV_8UC3, image.buffer);

			ROS_DEBUG_STREAM("[RealSense ID]: Preview " << image.width << "x" << image.height << " (" << image.size << "B)");
		}
};

#endif

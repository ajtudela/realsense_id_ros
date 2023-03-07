/*
 * REALSENSE CALLBACKS CLASSES
 *
 * Copyright (c) 2021-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#ifndef REALSENSE_ID_ROS__REALSENSE_CALLBACKS_HPP_
#define REALSENSE_ID_ROS__REALSENSE_CALLBACKS_HPP_

// C++
#include <mutex>
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
#include <rclcpp/rclcpp.hpp>

#include "realsense_id_ros/detection_object.hpp"

class RSAuthenticationCallback: public RealSenseID::AuthenticationCallback{
	public:
		void clear(){
			detections_.clear();
			results_ = 0;
		};

		void setDeviceConfig(RealSenseID::DeviceConfig devConfig){device_config_ = devConfig;};

		void OnResult(const RealSenseID::AuthenticateStatus status, const char* userId) override{
			bool spoof = false;

			if (status == RealSenseID::AuthenticateStatus::Success){
				if (device_config_.algo_flow == RealSenseID::DeviceConfig::AlgoFlow::SpoofOnly){
					RCLCPP_INFO(rclcpp::get_logger("RealSenseID"), "Real face");
				}else{
					RCLCPP_INFO_STREAM(rclcpp::get_logger("RealSenseID"), "Authenticated, userdId = " 
										<< userId);
				}
			}else if (status == RealSenseID::AuthenticateStatus::Forbidden){
				RCLCPP_INFO(rclcpp::get_logger("RealSenseID"), "User is not authenticated");
			}else if (status == RealSenseID::AuthenticateStatus::Spoof){
				spoof = true;
				RCLCPP_INFO(rclcpp::get_logger("RealSenseID"), "Spoof");
			}else if (status == RealSenseID::AuthenticateStatus::NoFaceDetected){
				RCLCPP_DEBUG(rclcpp::get_logger("RealSenseID"), "NoFaceDetected");
			}else{
				RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Authenticate " << status);
			}

			// Check results and add them to objects
			if (faces_.size() > results_){
				auto& face = faces_[results_];

				// Create new detection
				DetectionObject new_detection;
				new_detection.x = face.x;
				new_detection.y = face.y;
				new_detection.width = face.w;
				new_detection.height = face.h;
				new_detection.has_mask = false;
				new_detection.id = spoof ? "Spoof": userId;
				new_detection.confidence = 1.0;
				detections_.push_back(new_detection);

				RCLCPP_DEBUG(rclcpp::get_logger("RealSenseID"), "Detected face %u,%u %ux%u", 
							face.x, face.y, face.w, face.h);
			}
			results_++;
		}

		void OnHint(const RealSenseID::AuthenticateStatus hint) override{
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Hint " << hint);
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
		RealSenseID::DeviceConfig device_config_;
		size_t results_ = 0;
		unsigned int ts_ = 0;
};

class RSEnrollmentCallback: public RealSenseID::EnrollmentCallback{
	public:
		void OnResult(const RealSenseID::EnrollStatus status) override{
			if (status == RealSenseID::EnrollStatus::Success){
				RCLCPP_INFO(rclcpp::get_logger("RealSenseID"), "Real face");
			}else if (status == RealSenseID::EnrollStatus::Spoof){
				RCLCPP_INFO(rclcpp::get_logger("RealSenseID"), "Spoof");
			}else if (status == RealSenseID::EnrollStatus::NoFaceDetected){
				RCLCPP_INFO(rclcpp::get_logger("RealSenseID"), "NoFaceDetected");
			}else{
				RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Result " << status);
			}

			// Check results and add them to objects
			if (faces_.size() > results_){
				auto& face = faces_[results_];

				// Create new detection
				DetectionObject new_detection;
				new_detection.x = face.x;
				new_detection.y = face.y;
				new_detection.width = face.w;
				new_detection.height = face.h;
				new_detection.confidence = 1.0;
				new_detection.has_mask = false;
				detections_.push_back(new_detection);

				RCLCPP_DEBUG(rclcpp::get_logger("RealSenseID"), "Detected face %u,%u %ux%u", 
							face.x, face.y, face.w, face.h);
			}
			results_++;
		}

		void OnProgress(const RealSenseID::FacePose pose) override{
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Progress " << pose);
		}

		void OnHint(const RealSenseID::EnrollStatus hint) override{
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Hint " << hint);
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
		RSPreviewCallback(std::mutex *mutex): mutex(mutex){}
		void OnPreviewImageReady(const RealSenseID::Image image){
			// Lock mutex
			std::lock_guard<std::mutex> lock(*mutex);
			// Convert to CV Mat
			image_ = cv::Mat(image.height, image.width, CV_8UC3, image.buffer);
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Preview " << 
							image.width << "x" << image.height << " (" << image.size << "B)");
		}

		const cv::Mat& GetImage(){
			return image_;
		}

	private:
		std::mutex *mutex;
		cv::Mat image_;
};

#endif  // REALSENSE_ID_ROS__REALSENSE_CALLBACKS_HPP_

/*
 * REALSENSE ID ROS CLASS
 *
 * Copyright (c) 2021-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#ifndef REALSENSE_ID_ROS__REALSENSE_ID_HPP_
#define REALSENSE_ID_ROS__REALSENSE_ID_HPP_

// C++
#include <mutex>
#include <string>
#include <thread>

// OpenCV
#include <opencv2/core.hpp>

// RealSense ID
#include <RealSenseID/DeviceController.h>
#include <RealSenseID/DeviceConfig.h>
#include <RealSenseID/FaceAuthenticator.h>
#include <RealSenseID/Logging.h>
#include <RealSenseID/Preview.h>
#include <RealSenseID/Version.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "face_msgs/msg/face_array.hpp"
#include "face_msgs/srv/authenticate.hpp"
#include "face_msgs/srv/device_info.hpp"
#include "face_msgs/srv/enroll.hpp"
#include "face_msgs/srv/query_users_id.hpp"
#include "face_msgs/srv/remove_user.hpp"
#include "face_msgs/srv/remove_all_users.hpp"

#include "realsense_id_ros/realsense_callbacks.hpp"
#include "realsense_id_ros/realsense_server_callbacks.hpp"
#include "realsense_id_ros/faceprints_database.hpp"

class RealSenseIDROS: public rclcpp::Node{
	public:
		RealSenseIDROS();
		~RealSenseIDROS();

	private:
		rclcpp::Publisher<face_msgs::msg::FaceArray>::SharedPtr faces_pub_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
		rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

		rclcpp::Service<face_msgs::srv::DeviceInfo>::SharedPtr get_device_info_service_;
		rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_service_;

		rclcpp::Service<face_msgs::srv::Authenticate>::SharedPtr auth_service_;
		rclcpp::Service<face_msgs::srv::Enroll>::SharedPtr enroll_service_;
		rclcpp::Service<face_msgs::srv::RemoveUser>::SharedPtr remove_user_service_;
		rclcpp::Service<face_msgs::srv::RemoveAllUsers>::SharedPtr remove_all_user_service_;
		rclcpp::Service<face_msgs::srv::QueryUsersId>::SharedPtr query_users_service_;

		OnSetParametersCallbackHandle::SharedPtr callback_handle_;

		std::mutex mutex_;
		bool setup_, restore_, server_mode_;
		std::string port_, db_filepath_, frame_id_;
		cv::Mat preview_cv_image_;
		rclcpp::TimerBase::SharedPtr timer_;
		sensor_msgs::msg::CameraInfo camera_info_;

		// RealSense ID
		RealSenseID::SerialConfig serial_config_;
		RealSenseID::DeviceConfig device_config_;
		RealSenseID::PreviewConfig preview_config_;
		RSAuthenticationCallback auth_clbk_;
		RSAuthFaceprintsCallback auth_face_clbk_;
		FaceprintsDatabase faceprints_db_;
		std::unique_ptr<RealSenseID::Preview> preview_;
		std::unique_ptr<RSPreviewCallback> preview_clbk_;

		void get_params();
		void update();
		std::unique_ptr<RealSenseID::FaceAuthenticator> create_authenticator(const RealSenseID::SerialConfig& serial_config);
		void log_callback(RealSenseID::LogLevel level, const char* msg);
		rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);

		bool get_device_info(const std::shared_ptr<face_msgs::srv::DeviceInfo::Request> req, 
								std::shared_ptr<face_msgs::srv::DeviceInfo::Response> res);
		bool set_camera_info(const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req, 
								std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> res);

		// Services - Device mode
		bool authenticate_service(const std::shared_ptr<face_msgs::srv::Authenticate::Request> req,
									std::shared_ptr<face_msgs::srv::Authenticate::Response> res);
		bool enroll_service(const std::shared_ptr<face_msgs::srv::Enroll::Request> req, 
									std::shared_ptr<face_msgs::srv::Enroll::Response> res);
		bool remove_user_service(const std::shared_ptr<face_msgs::srv::RemoveUser::Request> req, 
									std::shared_ptr<face_msgs::srv::RemoveUser::Response> res);
		bool remove_all_service(const std::shared_ptr<face_msgs::srv::RemoveAllUsers::Request> req, 
									std::shared_ptr<face_msgs::srv::RemoveAllUsers::Response> res);
		bool query_users_id_service(const std::shared_ptr<face_msgs::srv::QueryUsersId::Request> req, 
									std::shared_ptr<face_msgs::srv::QueryUsersId::Response> res);

		// Services - Host mode
		bool authenticate_faceprints_service(const std::shared_ptr<face_msgs::srv::Authenticate::Request> req, 
								std::shared_ptr<face_msgs::srv::Authenticate::Response> res);
		bool enroll_faceprints_service(const std::shared_ptr<face_msgs::srv::Enroll::Request> req, 
								std::shared_ptr<face_msgs::srv::Enroll::Response> res);
		bool remove_user_faceprints_service(const std::shared_ptr<face_msgs::srv::RemoveUser::Request> req, 
								std::shared_ptr<face_msgs::srv::RemoveUser::Response> res);
		bool remove_all_faceprints_service(const std::shared_ptr<face_msgs::srv::RemoveAllUsers::Request> req, 
								std::shared_ptr<face_msgs::srv::RemoveAllUsers::Response> res);
		bool query_users_id_faceprints_service(const std::shared_ptr<face_msgs::srv::QueryUsersId::Request> req, 
								std::shared_ptr<face_msgs::srv::QueryUsersId::Response> res);
};

#endif  // REALSENSE_ID_ROS__REALSENSE_ID_HPP_

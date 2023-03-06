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

// OpenCV
#include <cv_bridge/cv_bridge.h>

// ROS
#include "nav2_util/node_utils.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/fill_image.hpp"

#include "realsense_id_ros/msg/face_array.hpp"
#include "realsense_id_ros/realsense_id_ros.hpp"
#include "realsense_id_ros/realsense_server_callbacks.hpp"

/* Initialize the subscribers and the publishers */
RealSenseIDROS::RealSenseIDROS() : Node("realsense_id_ros"), running_(false){
	// Initialize ROS parameters
	get_params();

	// Set log level
	RealSenseID::SetLogCallback(
		std::bind(&RealSenseIDROS::log_callback, this, std::placeholders::_1, std::placeholders::_2), 
		RealSenseID::LogLevel::Off, true);

	// Set serial config and connect
	serial_config_.port = port_.c_str();
	authenticator_ = std::make_unique<RealSenseID::FaceAuthenticator>();
	auto connect_status = authenticator_->Connect(serial_config_);
	if (connect_status != RealSenseID::Status::Ok){
		RCLCPP_ERROR_STREAM(this->get_logger(), 
			"Failed connecting to port " << serial_config_.port << " status:" << connect_status);
		exit(1);
	}else{
		RCLCPP_INFO_STREAM(this->get_logger(), "Connected to device in port" << serial_config_.port);
		RCLCPP_DEBUG_STREAM(this->get_logger(), "Opening serial port " << port_.c_str());
	}

	// Set device config
	auto status = authenticator_->SetDeviceConfig(device_config_);
	if (status != RealSenseID::Status::Ok){
		RCLCPP_ERROR(this->get_logger(), "Failed to apply device settings!");
		authenticator_->Disconnect();
	}

	// Callback for monitor changes in parameters
	callback_handle_ = this->add_on_set_parameters_callback(
						std::bind(&RealSenseIDROS::parameters_callback, this, std::placeholders::_1));

	// Initialize services
	get_device_info_service_ = this->create_service<realsense_id_ros::srv::DeviceInfo>("device_info", 
									std::bind(&RealSenseIDROS::get_device_info, this, std::placeholders::_1, std::placeholders::_2));
	set_camera_info_service_ = this->create_service<sensor_msgs::srv::SetCameraInfo>("set_camera_info", 
									std::bind(&RealSenseIDROS::set_camera_info, this, std::placeholders::_1, std::placeholders::_2));
	start_auth_loop_service_ = this->create_service<realsense_id_ros::srv::StartAuthenticationLoop>("start_authentication_loop", 
									std::bind(&RealSenseIDROS::start_authentication_loop, this, std::placeholders::_1, std::placeholders::_2));
	stop_auth_loop_service_ = this->create_service<realsense_id_ros::srv::StopAuthenticationLoop>("stop_authentication_loop", 
									std::bind(&RealSenseIDROS::stop_authentication_loop, this, std::placeholders::_1, std::placeholders::_2));

	if (!server_mode_){
		RCLCPP_INFO(this->get_logger(), "Using API in device mode");
		auth_service_ = this->create_service<realsense_id_ros::srv::Authenticate>("authenticate", 
							std::bind(&RealSenseIDROS::authenticate_service, this, std::placeholders::_1, std::placeholders::_2));
		enroll_service_ = this->create_service<realsense_id_ros::srv::Enroll>("enroll", 
							std::bind(&RealSenseIDROS::enroll_service, this, std::placeholders::_1, std::placeholders::_2));
		remove_user_service_ = this->create_service<realsense_id_ros::srv::RemoveUser>("remove_user",
							std::bind(&RealSenseIDROS::remove_user_service, this, std::placeholders::_1, std::placeholders::_2));
		remove_all_user_service_ = this->create_service<realsense_id_ros::srv::RemoveAllUsers>("remove_all_users",
							std::bind(&RealSenseIDROS::remove_all_service, this, std::placeholders::_1, std::placeholders::_2));
		query_users_service_ = this->create_service<realsense_id_ros::srv::QueryUsersId>("query_users_id",
							std::bind(&RealSenseIDROS::query_users_id_service, this, std::placeholders::_1, std::placeholders::_2));
	}else{
		RCLCPP_INFO(this->get_logger(), "Using API in server mode");
		auth_service_ = this->create_service<realsense_id_ros::srv::Authenticate>("authenticate", 
							std::bind(&RealSenseIDROS::authenticate_faceprints_service, this, std::placeholders::_1, std::placeholders::_2));
		enroll_service_ = this->create_service<realsense_id_ros::srv::Enroll>("enroll", 
							std::bind(&RealSenseIDROS::enroll_faceprints_service, this, std::placeholders::_1, std::placeholders::_2));
		remove_user_service_ = this->create_service<realsense_id_ros::srv::RemoveUser>("remove_user",
							std::bind(&RealSenseIDROS::remove_user_faceprints_service, this, std::placeholders::_1, std::placeholders::_2));
		remove_all_user_service_ = this->create_service<realsense_id_ros::srv::RemoveAllUsers>("remove_all_users",
							std::bind(&RealSenseIDROS::remove_all_faceprints_service, this, std::placeholders::_1, std::placeholders::_2));
		query_users_service_ = this->create_service<realsense_id_ros::srv::QueryUsersId>("query_users_id",
							std::bind(&RealSenseIDROS::query_users_id_faceprints_service, this, std::placeholders::_1, std::placeholders::_2));
	}

	// And publishers
	faces_pub_              = this->create_publisher<realsense_id_ros::msg::FaceArray>("faces", 1);
	image_pub_              = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
	camera_info_pub_        = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);

	// And timer
	timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&RealSenseIDROS::update, this));

	// Load the database from a file
	if (server_mode_){
		if (!db_filepath_.empty()){
			faceprints_db_.load_db_from_file(db_filepath_);
			RCLCPP_INFO(this->get_logger(), "Faceprints database load from file");
		}
	}

	// Change authenticate loop
	if (!running_ && auth_loop_mode_){
		std::shared_ptr<realsense_id_ros::srv::StartAuthenticationLoop::Request> request;
		std::shared_ptr<realsense_id_ros::srv::StartAuthenticationLoop::Response> response;
		start_authentication_loop(request, response);
	}

	preview_.reset();
}

/* Delete all parameteres-> */
RealSenseIDROS::~RealSenseIDROS(){
	// Cancel
	authenticator_->Cancel();

	// Disconnect from the device
	authenticator_->Disconnect();

	// Stop preview
	preview_->StopPreview();
}

/* Update parameters of the node. */
void RealSenseIDROS::get_params(){
	RCLCPP_INFO(this->get_logger(), "Reading ROS parameters");

	// BOOLEAN PARAMS ..........................................................................
	nav2_util::declare_parameter_if_not_declared(this, "authenticate_loop", rclcpp::ParameterValue(false), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Start authentication loop"));
	this->get_parameter("authenticate_loop", auth_loop_mode_);
	RCLCPP_INFO(this->get_logger(), "The parameter authenticate_loop is set to: [%s]", auth_loop_mode_ ? "true" : "false");

	nav2_util::declare_parameter_if_not_declared(this, "server_mode", rclcpp::ParameterValue(false), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Use API in server mode"));
	this->get_parameter("server_mode", server_mode_);
	RCLCPP_INFO(this->get_logger(), "The parameter server_mode is set to: [%s]", server_mode_ ? "true" : "false");

	nav2_util::declare_parameter_if_not_declared(this, "restore_defaults", rclcpp::ParameterValue(false), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Restore to the original configuration"));
	this->get_parameter("restore_defaults", restore_);
	RCLCPP_INFO(this->get_logger(), "The parameter restore_defaults is set to: [%s]", restore_ ? "true" : "false");

	// INTEGER PARAMS ..........................................................................
	nav2_util::declare_parameter_if_not_declared(this, "camera_rotation", rclcpp::ParameterValue(0), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Rotate the image inside the device (0, 90, 180, 270)")
							.set__integer_range({rcl_interfaces::msg::IntegerRange()
								.set__from_value(0)
								.set__to_value(270)
								.set__step(90)}
								));
	int camera_rotation;
	this->get_parameter("camera_rotation", camera_rotation);
	if (camera_rotation == 0){
		device_config_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_0_Deg;
	}else if (camera_rotation == 180){
		device_config_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_180_Deg;
	}else if (camera_rotation == 90){
		device_config_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_90_Deg;
	}else if (camera_rotation == 270){
		device_config_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_270_Deg;
	}
	RCLCPP_INFO(this->get_logger(), "The parameter camera_rotation is set to: [%d]", camera_rotation);

	// STRING PARAMS ..........................................................................
	nav2_util::declare_parameter_if_not_declared(this, "serial_port", rclcpp::ParameterValue("/dev/ttyACM0"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Serial port of the device"));
	this->get_parameter("serial_port", port_);
	RCLCPP_INFO(this->get_logger(), "The parameter serial_port is set to: [%s]", port_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "database_filepath", rclcpp::ParameterValue(""), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Path to the database file"));
	this->get_parameter("database_filepath", db_filepath_);
	RCLCPP_INFO(this->get_logger(), "The parameter database_filepath is set to: [%s]", db_filepath_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "frame_id", rclcpp::ParameterValue("realsense_id_optical_frame"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Frame id of the camera"));
	this->get_parameter("frame_id", frame_id_);
	RCLCPP_INFO(this->get_logger(), "The parameter frame_id is set to: [%s]", frame_id_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "security_level", rclcpp::ParameterValue("medium"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Set 'high' to allow no mask suport or 'medium' to support masks"));
	std::string security_level;
	this->get_parameter("security_level", security_level);
	if (security_level.find("high") != std::string::npos){
		device_config_.security_level = RealSenseID::DeviceConfig::SecurityLevel::High;
	}else if (security_level.find("medium") != std::string::npos){
		device_config_.security_level = RealSenseID::DeviceConfig::SecurityLevel::Medium;
	}else if (security_level.find("low") != std::string::npos){
		device_config_.security_level = RealSenseID::DeviceConfig::SecurityLevel::Low;
	}
	
	RCLCPP_INFO(this->get_logger(), "The parameter security_level is set to: [%s]", security_level.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "algo_flow", rclcpp::ParameterValue("all"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Algorithms used during authentication: 'all', 'detection', 'recognition' and 'spoof'"));
	std::string algo_flow;
	this->get_parameter("algo_flow", algo_flow);
	if (algo_flow.find("all") != std::string::npos){
		device_config_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::All;
	}else if (algo_flow.find("detection") != std::string::npos){
		device_config_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::FaceDetectionOnly;
	}else if (algo_flow.find("spoof") != std::string::npos){
		device_config_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::SpoofOnly;
	}else if (algo_flow.find("recognition") != std::string::npos){
		device_config_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::RecognitionOnly;
	}
	RCLCPP_INFO(this->get_logger(), "The parameter algo_flow is set to: [%s]", algo_flow.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "face_selection_policy", rclcpp::ParameterValue("all"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Run authentication on 'all' (up to 5) detected faces vs 'single' (closest) face"));
	
	std::string face_selection_policy;
	this->get_parameter("face_selection_policy", face_selection_policy);
	if (face_selection_policy.find("single") != std::string::npos){
		device_config_.face_selection_policy = RealSenseID::DeviceConfig::FaceSelectionPolicy::Single;
	}else if (face_selection_policy.find("all") != std::string::npos){
		device_config_.face_selection_policy = RealSenseID::DeviceConfig::FaceSelectionPolicy::All;
	}
	RCLCPP_INFO(this->get_logger(), "The parameter face_selection_policy is set to: [%s]", face_selection_policy.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "dump_mode", rclcpp::ParameterValue("none"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Dump image: 'none', 'cropped' or 'fullframe'"));
	std::string dump_mode;
	this->get_parameter("dump_mode", dump_mode);
	if (dump_mode.find("none") != std::string::npos){
		device_config_.dump_mode = RealSenseID::DeviceConfig::DumpMode::None;
	}else if (dump_mode.find("cropped") != std::string::npos){
		device_config_.dump_mode = RealSenseID::DeviceConfig::DumpMode::CroppedFace;
	}else if (dump_mode.find("fullframe") != std::string::npos){
		device_config_.dump_mode = RealSenseID::DeviceConfig::DumpMode::FullFrame;
	}
	RCLCPP_INFO(this->get_logger(), "The parameter dump_mode is set to: [%s]", dump_mode.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "matcher_confidence_level", rclcpp::ParameterValue("high"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Matcher during authentication: 'high', 'medium' or 'low'"));
	std::string matcher_confidence_level;
	this->get_parameter("matcher_confidence_level", matcher_confidence_level);
	if (matcher_confidence_level.find("high") != std::string::npos){
		device_config_.matcher_confidence_level = RealSenseID::DeviceConfig::MatcherConfidenceLevel::High;
	}else if (matcher_confidence_level.find("medium") != std::string::npos){
		device_config_.matcher_confidence_level = RealSenseID::DeviceConfig::MatcherConfidenceLevel::Medium;
	}else if (matcher_confidence_level.find("low") != std::string::npos){
		device_config_.matcher_confidence_level = RealSenseID::DeviceConfig::MatcherConfidenceLevel::Low;
	}
	RCLCPP_INFO(this->get_logger(), "The parameter matcher_confidence_level is set to: [%s]", matcher_confidence_level.c_str());
}

/* Reconfigure callback. */
rcl_interfaces::msg::SetParametersResult RealSenseIDROS::parameters_callback(const std::vector<rclcpp::Parameter> &parameters){
	RCLCPP_INFO(this->get_logger(), "Reconfigure request for RealSenseID");
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";

	for (const auto &param: parameters){
		// BOOLEAN PARAMS ..........................................................................
		if (param.get_name() == "authenticate_loop" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
				auth_loop_mode_ = param.as_bool();
				RCLCPP_INFO(this->get_logger(), "The parameter authenticate_loop is set to: [%s]", auth_loop_mode_ ? "true" : "false");
		}
		if (param.get_name() == "server_mode" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
				server_mode_ = param.as_bool();
				RCLCPP_INFO(this->get_logger(), "The parameter server_mode is set to: [%s]", server_mode_ ? "true" : "false");
		}
		if (param.get_name() == "restore_defaults" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
				restore_ = param.as_bool();
				RCLCPP_INFO(this->get_logger(), "The parameter restore_defaults is set to: [%s]", restore_ ? "true" : "false");
		}
		// INTEGER PARAMS ..........................................................................
		if (param.get_name() == "camera_rotation" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER){
			int camera_rotation = param.as_int();
			if (camera_rotation == 0){
				device_config_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_0_Deg;
			}else if (camera_rotation == 180){
				device_config_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_180_Deg;
			}else if (camera_rotation == 90){
				device_config_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_90_Deg;
			}else if (camera_rotation == 270){
				device_config_.camera_rotation = RealSenseID::DeviceConfig::CameraRotation::Rotation_270_Deg;
			}
			RCLCPP_INFO(this->get_logger(), "The parameter camera_rotation is set to: [%d]", camera_rotation);
		}

		// STRING PARAMS ...........................................................................
		if (param.get_name() == "security_level" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
			std::string security_level = param.as_string();
			if (security_level.find("high") != std::string::npos){
				device_config_.security_level = RealSenseID::DeviceConfig::SecurityLevel::High;
			}else if (security_level.find("medium") != std::string::npos){
				device_config_.security_level = RealSenseID::DeviceConfig::SecurityLevel::Medium;
			}else if (security_level.find("low") != std::string::npos){
				device_config_.security_level = RealSenseID::DeviceConfig::SecurityLevel::Low;
			}
			RCLCPP_INFO(this->get_logger(), "The parameter security_level is set to: [%s]", security_level.c_str());
		}
		if (param.get_name() == "algo_flow" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
			std::string algo_flow = param.as_string();
			if (algo_flow.find("all") != std::string::npos){
				device_config_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::All;
			}else if (algo_flow.find("detection") != std::string::npos){
				device_config_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::FaceDetectionOnly;
			}else if (algo_flow.find("spoof") != std::string::npos){
				device_config_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::SpoofOnly;
			}else if (algo_flow.find("recognition") != std::string::npos){
				device_config_.algo_flow = RealSenseID::DeviceConfig::AlgoFlow::RecognitionOnly;
			}
			RCLCPP_INFO(this->get_logger(), "The parameter algo_flow is set to: [%s]", algo_flow.c_str());
		}
		if (param.get_name() == "face_selection_policy" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
			std::string face_selection_policy = param.as_string();
			if (face_selection_policy.find("single") != std::string::npos){
				device_config_.face_selection_policy = RealSenseID::DeviceConfig::FaceSelectionPolicy::Single;
			}else if (face_selection_policy.find("all") != std::string::npos){
				device_config_.face_selection_policy = RealSenseID::DeviceConfig::FaceSelectionPolicy::All;
			}
			RCLCPP_INFO(this->get_logger(), "The parameter face_selection_policy is set to: [%s]", face_selection_policy.c_str());
		}
		if (param.get_name() == "dump_mode" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
			std::string dump_mode = param.as_string();
			if (dump_mode.find("none") != std::string::npos){
				device_config_.dump_mode = RealSenseID::DeviceConfig::DumpMode::None;
			}else if (dump_mode.find("cropped") != std::string::npos){
				device_config_.dump_mode = RealSenseID::DeviceConfig::DumpMode::CroppedFace;
			}else if (dump_mode.find("fullframe") != std::string::npos){
				device_config_.dump_mode = RealSenseID::DeviceConfig::DumpMode::FullFrame;
			}
			RCLCPP_INFO(this->get_logger(), "The parameter dump_mode is set to: [%s]", dump_mode.c_str());
		}
		if (param.get_name() == "matcher_confidence_level" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
			std::string matcher_confidence_level = param.as_string();
			if (matcher_confidence_level.find("high") != std::string::npos){
				device_config_.matcher_confidence_level = RealSenseID::DeviceConfig::MatcherConfidenceLevel::High;
			}else if (matcher_confidence_level.find("medium") != std::string::npos){
				device_config_.matcher_confidence_level = RealSenseID::DeviceConfig::MatcherConfidenceLevel::Medium;
			}else if (matcher_confidence_level.find("low") != std::string::npos){
				device_config_.matcher_confidence_level = RealSenseID::DeviceConfig::MatcherConfidenceLevel::Low;
			}
			RCLCPP_INFO(this->get_logger(), "The parameter matcher_confidence_level is set to: [%s]", matcher_confidence_level.c_str());
		}
	}

	auto status = authenticator_->SetDeviceConfig(device_config_);
	if (status != RealSenseID::Status::Ok){
		RCLCPP_ERROR(this->get_logger(), "Failed to apply device settings!");
		authenticator_->Disconnect();
	}

	return result;
}

/* Logging callback. */
void RealSenseIDROS::log_callback(RealSenseID::LogLevel level, const char* msg){
	switch (level){
		case RealSenseID::LogLevel::Debug:
			RCLCPP_DEBUG_STREAM(this->get_logger(), "" << msg);
			break;
		case RealSenseID::LogLevel::Warning:
			RCLCPP_WARN_STREAM(this->get_logger(), "" << msg);
			break;
		case RealSenseID::LogLevel::Error:
		case RealSenseID::LogLevel::Critical:
			RCLCPP_ERROR_STREAM(this->get_logger(), "" << msg);
			break;
		case RealSenseID::LogLevel::Info:
		default:
			RCLCPP_INFO_STREAM(this->get_logger(), "" << msg);
			break;
	}
}

/* Authenticate loop */
void RealSenseIDROS::authenticate_loop(){
	if (!server_mode_){
		authenticator_->AuthenticateLoop(auth_clbk_);
	}else{
		auth_face_clbk_.setAuthenticator(authenticator_);
		auth_face_clbk_.setFaceprintsDatabase(faceprints_db_.data);
		authenticator_->ExtractFaceprintsForAuthLoop(auth_face_clbk_);
	}
}

/* Publish Faces */
void RealSenseIDROS::update(){
	if (!auth_loop_mode_) return;

	// Create header of FaceArray
	realsense_id_ros::msg::FaceArray face_array;
	face_array.header.frame_id = frame_id_;
	face_array.header.stamp = this->now();

	// Get image
	preview_cv_image_ = preview_clbk_.GetImage();
	const size_t color_height = (size_t) preview_cv_image_.size().height;
	const size_t color_width  = (size_t) preview_cv_image_.size().width;

	// Create face array message
	std::vector<DetectionObject> detections;
	if (!server_mode_){
		detections = auth_clbk_.GetDetections();
	}else{
		detections = auth_face_clbk_.GetDetections();
	}

	for (auto &detection: detections){
		// Sanitize and convert to face msg
		detection.sanitize_size(color_width, color_height);
		auto face = detection.to_msg(face_array.header, preview_cv_image_);
		face_array.faces.push_back(face);

		// Color of the person
		cv::Scalar color;
		if (detection.id == realsense_id_ros::msg::Face::SPOOF || detection.id.empty()) color = cv::Scalar(255, 0, 0);
		else color = cv::Scalar(0, 255, 0);
		// Text label
		std::ostringstream conf;
		conf << ":" << std::fixed << std::setprecision(3) << detection.confidence;
		std::string labelText = detection.id + conf.str();
		// Rectangles for faces
		cv::rectangle(preview_cv_image_, cv::Point2f(detection.x-1, detection.y), cv::Point2f(detection.x + 400, detection.y - 40), color, cv::FILLED, cv::LINE_AA);
		cv::putText(preview_cv_image_, labelText, cv::Point2f(detection.x, detection.y - 5), cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 0), 1.5, cv::LINE_AA);
		cv::rectangle(preview_cv_image_, cv::Point2f(detection.x, detection.y), cv::Point2f(detection.x + detection.width, detection.y + detection.height), color, 4, cv::LINE_AA);
	}

	// Publish array of faces
	faces_pub_->publish(face_array);

	// Clear
	if (!server_mode_){
		auth_clbk_.clear();
	}else{
		auth_face_clbk_.clear();
	}

	// Convert CV image to sensor msgs
	cv_bridge::CvImage output_msg;
	output_msg.header = face_array.header;
	output_msg.encoding = sensor_msgs::image_encodings::RGB8;
	output_msg.image = preview_cv_image_;
	image_pub_->publish(*output_msg.toImageMsg());

	//Publish camera info
	camera_info_.header = face_array.header;
	camera_info_.height = color_height;
	camera_info_.width = color_width;
	camera_info_.distortion_model = "plumb_bob";
	camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
	camera_info_.k = {911.9729056029453, 0.0, 543.4705406497254, 0.0, 935.5803580030122, 902.0450795440844, 0.0, 0.0, 1.0};
	camera_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	camera_info_.p = {999.5663452148438, 0.0, 557.4771347235219, 0.0, 0.0, 980.9320678710938, 835.6312784614311, 0.0, 0.0, 0.0, 1.0, 0.0};
	camera_info_pub_->publish(camera_info_);
}

/* Start authentication loop */
bool RealSenseIDROS::start_authentication_loop(const std::shared_ptr<realsense_id_ros::srv::StartAuthenticationLoop::Request> req, 
								std::shared_ptr<realsense_id_ros::srv::StartAuthenticationLoop::Response> res){
	RCLCPP_INFO(this->get_logger(), "Start authentication loop service request");

	if (!running_){
		preview_ = std::make_unique<RealSenseID::Preview>(preview_config_);
		preview_->StartPreview(preview_clbk_);
		auth_loop_thread_ = std::thread(std::bind(&RealSenseIDROS::authenticate_loop, this));
		running_ = true;
		auth_loop_mode_ = true;
		this->set_parameter(rclcpp::Parameter("authenticate_loop", auth_loop_mode_));
		RCLCPP_INFO_ONCE(this->get_logger(), "Authentication loop started");
		return true;
	}else{
		RCLCPP_ERROR(this->get_logger(), "Cannot start the authentication loop");
		return false;
	}
}

/* Cancel authentication loop */
bool RealSenseIDROS::stop_authentication_loop(const std::shared_ptr<realsense_id_ros::srv::StopAuthenticationLoop::Request> req, 
								std::shared_ptr<realsense_id_ros::srv::StopAuthenticationLoop::Response> res){
	RCLCPP_INFO(this->get_logger(), "Cancel authentication loop service request");

	if (running_){
		authenticator_->Cancel();
		preview_->StopPreview();
		auth_loop_thread_.join();
		running_ = false;
		auth_loop_mode_ = false;
		this->set_parameter(rclcpp::Parameter("authenticate_loop", auth_loop_mode_));
		RCLCPP_INFO_ONCE(this->get_logger(), "Authentication loop stopped");
		return true;
	}else{
		this->set_parameter(rclcpp::Parameter("authenticate_loop", auth_loop_mode_));
		RCLCPP_ERROR(this->get_logger(), "Cannot stop the authentication loop");
		return false;
	}
}

/* Get device info. */
bool RealSenseIDROS::get_device_info(const std::shared_ptr<realsense_id_ros::srv::DeviceInfo::Request> req, 
								std::shared_ptr<realsense_id_ros::srv::DeviceInfo::Response> res){
	RCLCPP_INFO(this->get_logger(), "Get device info service request");

	RealSenseID::DeviceController device_controller;

	auto connect_status = device_controller.Connect(serial_config_);
	if (connect_status != RealSenseID::Status::Ok){
		RCLCPP_INFO_STREAM(this->get_logger(), 
			"Failed connecting to port " << serial_config_.port << " status:" << connect_status);
		return false;
	}

	std::string firmware_version;
	auto status = device_controller.QueryFirmwareVersion(firmware_version);
	if (status != RealSenseID::Status::Ok){
		RCLCPP_INFO(this->get_logger(), "Failed getting firmware version!");
		return false;
	}

	std::string serial_number;
	status = device_controller.QuerySerialNumber(serial_number);
	if (status != RealSenseID::Status::Ok){
		RCLCPP_INFO(this->get_logger(), "Failed getting serial number!");
		return false;
	}

	device_controller.Disconnect();

	std::string host_version = RealSenseID::Version();

	res->device_name = "Intel Realsense F450 / F455";
	res->serial_number = serial_number;
	res->firmware_version = firmware_version;
	res->host_version = host_version;

	return true;
}

/* Set camera info */
bool RealSenseIDROS::set_camera_info(const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req, 
								std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> res){
	RCLCPP_INFO(this->get_logger(), "Set camera info service request");

	camera_info_ = req->camera_info;

	if (!rclcpp::ok()){
		RCLCPP_ERROR(this->get_logger(), "Camera driver not running.");
		res->status_message = "Camera driver not running.";
		res->success = false;
		return false;
	}

	return true;
}

// -------------------- DEVICE MODE ---------------

/* Perform one authentication. */
bool RealSenseIDROS::authenticate_service(const std::shared_ptr<realsense_id_ros::srv::Authenticate::Request> req,
									std::shared_ptr<realsense_id_ros::srv::Authenticate::Response> res){
	RCLCPP_INFO(this->get_logger(), "Authenticate service request");

	bool success = false;

	// Start preview
	preview_ = std::make_unique<RealSenseID::Preview>(preview_config_);
	preview_->StartPreview(preview_clbk_);

	// Get image
	preview_cv_image_ = preview_clbk_.GetImage();
	const size_t color_height = (size_t) preview_cv_image_.size().height;
	const size_t color_width  = (size_t) preview_cv_image_.size().width;

	// Authenticate a user
	RSAuthenticationCallback auth_clbk;
	auto status = authenticator_->Authenticate(auth_clbk);

	// Get timestamps saved in callbacks
	auto face_detection_ts = auth_clbk.GetLastTimeStamp();

	// Detections ...
	if (status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = auth_clbk.GetDetections();

		// Exit if no faces are detected
		if (detections.empty()){
			preview_->StopPreview();
			return false;
		}

		// Create header
		std_msgs::msg::Header header;
		header.frame_id = frame_id_;
		header.stamp = this->now();

		// Create face array message
		std::vector<realsense_id_ros::msg::Face> faces;
		for (auto &detection: detections){
			// Sanitize and convert to face msg
			detection.sanitize_size(color_width, color_height);
			auto face = detection.to_msg(header, preview_cv_image_);
			faces.push_back(face);
		}

		res->faces = faces;
		success = true;
	}

	// Stop preview
	preview_->StopPreview();

	return success;
}

/* Perform one enrollment for one new user. */
bool RealSenseIDROS::enroll_service(const std::shared_ptr<realsense_id_ros::srv::Enroll::Request> req, 
									std::shared_ptr<realsense_id_ros::srv::Enroll::Response> res){
	RCLCPP_INFO(this->get_logger(), "Enroll service request");

	bool success = false;

	// Start preview
	preview_ = std::make_unique<RealSenseID::Preview>(preview_config_);
	preview_->StartPreview(preview_clbk_);

	// Get image
	preview_cv_image_ = preview_clbk_.GetImage();
	const size_t color_height = (size_t) preview_cv_image_.size().height;
	const size_t color_width  = (size_t) preview_cv_image_.size().width;

	// Enroll a user
	RSEnrollmentCallback enroll_clbk;
	auto status = authenticator_->Enroll(enroll_clbk, req->id.c_str());

	// Create header
	std_msgs::msg::Header header;
	header.frame_id = frame_id_;
	header.stamp = this->now();

	// Detections ...
	if (status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = enroll_clbk.GetDetections();

		// Create face array message
		std::vector<realsense_id_ros::msg::Face> faces;
		for (auto &detection: detections){
			// Sanitize and convert to face msg
			detection.sanitize_size(color_width, color_height);
			auto face = detection.to_msg(header, preview_cv_image_);
			face.id = req->id.c_str();
			faces.push_back(face);
		}

		res->faces = faces;
		success = true;
	}

	// Stop preview
	preview_->StopPreview();

	return success;
}

/* Attempt to remove specific user from the device. */
bool RealSenseIDROS::remove_user_service(const std::shared_ptr<realsense_id_ros::srv::RemoveUser::Request> req, 
									std::shared_ptr<realsense_id_ros::srv::RemoveUser::Response> res){
	RCLCPP_INFO(this->get_logger(), "Remove user service request");

	// Remove a user
	auto status = authenticator_->RemoveUser(req->name.c_str());
	if (status == RealSenseID::Status::Ok){
		RCLCPP_INFO(this->get_logger(), "User removed successfully");
		return true;
	}else{
		RCLCPP_INFO(this->get_logger(), "User does not exists");
		return false;
	}
}

/* Attempt to remove all users from the device. */
bool RealSenseIDROS::remove_all_service(const std::shared_ptr<realsense_id_ros::srv::RemoveAllUsers::Request> req, 
									std::shared_ptr<realsense_id_ros::srv::RemoveAllUsers::Response> res){
	RCLCPP_INFO(this->get_logger(), "Remove all users service request");

	// Remove all users
	auto status = authenticator_->RemoveAll();
	if (status == RealSenseID::Status::Ok){
		RCLCPP_INFO(this->get_logger(), "Users removed successfully");
		return true;
	}else{
		RCLCPP_INFO(this->get_logger(), "Database is already empty");
		return false;
	}
}

/* Query the device about all enrolled users. */
bool RealSenseIDROS::query_users_id_service(const std::shared_ptr<realsense_id_ros::srv::QueryUsersId::Request> req, 
									std::shared_ptr<realsense_id_ros::srv::QueryUsersId::Response> res){
	RCLCPP_INFO(this->get_logger(), "Query users id service request");

	// Get number of users
	unsigned int number_of_users = 0;
	auto status = authenticator_->QueryNumberOfUsers(number_of_users);
	if (status != RealSenseID::Status::Ok){
		RCLCPP_INFO_STREAM(this->get_logger(), "Status: " << status);
		return false;
	}

	if (number_of_users == 0){
		RCLCPP_INFO(this->get_logger(), "No users found");
		return false;
	}

	// Allocate needed array of user ids
	char** user_ids = new char*[number_of_users];
	for(unsigned i = 0; i < number_of_users; i++){
		user_ids[i] = new char[RealSenseID::MAX_USERID_LENGTH];
	}
	unsigned int number_of_users_in_out = number_of_users;
	status = authenticator_->QueryUserIds(user_ids, number_of_users_in_out);
	if (status != RealSenseID::Status::Ok){
		RCLCPP_INFO_STREAM(this->get_logger(), "Status: " << status);
		// Free allocated memory and return on error
		for(unsigned int i = 0; i < number_of_users; i++){
			delete user_ids[i];
		}
		delete[] user_ids;
		return false;
	}

	// Create response
	res->number_users = number_of_users_in_out;
	for(unsigned int i = 0; i < number_of_users_in_out; i++){
		res->users_id.push_back(user_ids[i]);
	}

	// Free allocated memory
	for(unsigned int i = 0; i < number_of_users; i++){
		delete user_ids[i];
	}
	delete[] user_ids;

	return true;
}

// -------------------- SERVER MODE ---------------

/* Perform one authentication in server mode. */
bool RealSenseIDROS::authenticate_faceprints_service(const std::shared_ptr<realsense_id_ros::srv::Authenticate::Request> req, 
								std::shared_ptr<realsense_id_ros::srv::Authenticate::Response> res){
	RCLCPP_INFO(this->get_logger(), "Authenticate faceprints service request");

	bool success = false;

	// Start preview
	preview_ = std::make_unique<RealSenseID::Preview>(preview_config_);
	preview_->StartPreview(preview_clbk_);

	// Get image
	preview_cv_image_ = preview_clbk_.GetImage();
	const size_t color_height = (size_t) preview_cv_image_.size().height;
	const size_t color_width  = (size_t) preview_cv_image_.size().width;

	// Create callback
	RSAuthFaceprintsCallback auth_clbk(authenticator_, faceprints_db_.data);

	// Extract faceprints of the user in front of the device
	auto status = authenticator_->ExtractFaceprintsForAuth(auth_clbk);
	if (status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = auth_clbk.GetDetections();

		// Exit if no faces are detected
		if (detections.empty()){
			preview_->StopPreview();
			return false;
		}

		// Create header
		std_msgs::msg::Header header;
		header.frame_id = frame_id_;
		header.stamp = this->now();

		// Create face array message
		std::vector<realsense_id_ros::msg::Face> faces;
		for (auto &detection: detections){
			// Sanitize and convert to face msg
			detection.sanitize_size(color_width, color_height);
			auto face = detection.to_msg(header, preview_cv_image_);
			faces.push_back(face);
		}

		res->faces = faces;
		success = true;
	}

	// Stop preview
	preview_->StopPreview();

	return success;
}

/* Perform one enrollment for one new user in server mode. */
bool RealSenseIDROS::enroll_faceprints_service(const std::shared_ptr<realsense_id_ros::srv::Enroll::Request> req, 
								std::shared_ptr<realsense_id_ros::srv::Enroll::Response> res){
	RCLCPP_INFO(this->get_logger(), "Enroll faceprints service request");

	bool success = false;

	// Start preview
	preview_ = std::make_unique<RealSenseID::Preview>(preview_config_);
	preview_->StartPreview(preview_clbk_);

	// Get image
	preview_cv_image_ = preview_clbk_.GetImage();
	const size_t color_height = (size_t) preview_cv_image_.size().height;
	const size_t color_width  = (size_t) preview_cv_image_.size().width;

	// Create callback
	RSEnrollFaceprintsCallback enroll_clbk(req->id.c_str(), faceprints_db_.data);

	// Enroll a user
	auto status = authenticator_->ExtractFaceprintsForEnroll(enroll_clbk);
	if (status == RealSenseID::Status::Ok){
		std::vector<DetectionObject> detections = enroll_clbk.GetDetections();

		// Create header
		std_msgs::msg::Header header;
		header.frame_id = frame_id_;
		header.stamp = this->now();

		// Create face array message
		std::vector<realsense_id_ros::msg::Face> faces;
		for (auto &detection: detections){
			// Sanitize and convert to face msg
			detection.sanitize_size(color_width, color_height);
			auto face = detection.to_msg(header, preview_cv_image_);
			face.id = req->id.c_str();
			faces.push_back(face);
		}

		res->faces = faces;
		success = true;
	}

	// Stop preview
	preview_->StopPreview();

	// Save database to file
	faceprints_db_.data = enroll_clbk.getDatabase();

	return success;
}

/* Attempt to remove specific user from the database. */
bool RealSenseIDROS::remove_user_faceprints_service(const std::shared_ptr<realsense_id_ros::srv::RemoveUser::Request> req, 
								std::shared_ptr<realsense_id_ros::srv::RemoveUser::Response> res){
	RCLCPP_INFO(this->get_logger(), "Remove user faceprint service request");

	// Remove a user
	auto status = faceprints_db_.remove_user(req->name.c_str());
	if (status){
		RCLCPP_INFO(this->get_logger(), "User removed successfully");
		return true;
	}else{
		RCLCPP_INFO(this->get_logger(), "User does not exists");
		return false;
	}
}

/* Attempt to remove all users from the database. */
bool RealSenseIDROS::remove_all_faceprints_service(const std::shared_ptr<realsense_id_ros::srv::RemoveAllUsers::Request> req, 
								std::shared_ptr<realsense_id_ros::srv::RemoveAllUsers::Response> res){
	RCLCPP_INFO(this->get_logger(), "Remove all users faceprints service request");

	// Delete database
	faceprints_db_.remove_all_users();
	return true;
}

/* Query the device about all enrolled users in server mode. */
bool RealSenseIDROS::query_users_id_faceprints_service(const std::shared_ptr<realsense_id_ros::srv::QueryUsersId::Request> req, 
								std::shared_ptr<realsense_id_ros::srv::QueryUsersId::Response> res){
	RCLCPP_INFO(this->get_logger(), "Query users id faceprints service request");

	// Create response
	res->number_users = faceprints_db_.get_number_of_users();
	res->users_id = faceprints_db_.get_users();
	return true;
}

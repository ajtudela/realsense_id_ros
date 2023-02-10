/*
 * DETECTION OBJECT STRUCT
 *
 * Copyright (c) 2021-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#ifndef REALSENSE_ID_ROS__DETECTION_OBJECT_HPP_
#define REALSENSE_ID_ROS__DETECTION_OBJECT_HPP_

// C++
#include <string>

// OpenCV
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

// ROS
#include "std_msgs/msg/header.hpp"

#include "realsense_id_ros/msg/face.hpp"

struct DetectionObject{
	size_t x, y, width, height;
	float confidence;
	std::string id;
	bool has_mask;

	void sanitize_size(const size_t& imageWidth, const size_t& imageHeight){
		width = ((x + width) > imageWidth) ? (imageWidth - x) : width;
		height = ((y + height) > imageHeight) ? (imageHeight - y) : height;
	}

	/* Create a Face msg */
	realsense_id_ros::msg::Face to_msg(const std_msgs::msg::Header & header, const cv::Mat & image){
		realsense_id_ros::msg::Face face;

		// Header, id and confidence
		face.header = header;
		face.id = id;
		face.confidence = confidence;
		face.has_mask = has_mask;

		// 2D bounding box surrounding the object
		face.bbox.center.x = x + width / 2;
		face.bbox.center.y = y + height / 2;
		face.bbox.size_x = width;
		face.bbox.size_y = height;

		// The 2D data that generated these results
		if (!image.empty()){
			cv_bridge::CvImage output_msg;
			output_msg.header = header;
			output_msg.encoding = sensor_msgs::image_encodings::RGB8;
			output_msg.image = cv::Mat(image, cv::Rect(x, y, width, height));
			face.source_img = *output_msg.toImageMsg();
		}

		return face;
	}
};

#endif  // REALSENSE_ID_ROS__DETECTION_OBJECT_HPP

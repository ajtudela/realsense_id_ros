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

#include "face_msgs/msg/face.hpp"

struct DetectionObject{
	size_t x, y, width, height;
	float confidence;
	std::string id;
	bool has_mask;

	/* Create a Face msg */
	face_msgs::msg::Face to_msg(const std_msgs::msg::Header & header, const cv::Mat & image){
		face_msgs::msg::Face face;

		// Header, id and confidence
		face.header = header;
		face.id = id;
		face.score = confidence;
		face.has_mask = has_mask;

		// Resize the bounding box if it is out of the image
		width = ((x + width) > image.size().width) ? (image.size().width - x) : width;
		height = ((y + height) > image.size().height) ? (image.size().height - y) : height;

		// 2D bounding box surrounding the object
		face.bbox.center.x = x + width / 2;
		face.bbox.center.y = y + height / 2;
		face.bbox.size_x = width;
		face.bbox.size_y = height;

		return face;
	}
};

#endif  // REALSENSE_ID_ROS__DETECTION_OBJECT_HPP

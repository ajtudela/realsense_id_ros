/*
 * DETECTION OBJECT STRUCT
 *
 * Copyright (c) 2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#ifndef DETECTION_OBJECT_H
#define DETECTION_OBJECT_H

// C++
#include <string>

// OpenCV
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>

struct DetectionObject{
	size_t x, y, width, height;
	float confidence;
	std::string id;
	bool hasMask;

	void sanitizeSize(const size_t& imageWidth, const size_t& imageHeight){
		x = (x < 0) ? 0 : x;
		y = (y < 0) ? 0 : y;
		width = ((x + width) > imageWidth) ? (imageWidth - x) : width;
		height = ((y + height) > imageHeight) ? (imageHeight - y) : height;
	}

	/* Create a Face msg */
	realsense_id_ros::Face toFace(std_msgs::Header header, cv::Mat image){
		realsense_id_ros::Face face;

		// Header, id and confidence
		face.header = header;
		face.id = id;
		face.confidence = confidence;
		face.has_mask = hasMask;

		// 2D bounding box surrounding the object
		face.bbox.center.x = x + width / 2;
		face.bbox.center.y = y + height / 2;
		face.bbox.size_x = width;
		face.bbox.size_y = height;

		// The 2D data that generated these results
		if (!image.empty()){
			cv::Mat croppedImage = image(cv::Rect(x, y, width, height));
			cv_bridge::CvImage cvImageBr;
			cvImageBr.header.frame_id = header.frame_id;
			cvImageBr.header.stamp = ros::Time::now();
			cvImageBr.encoding = sensor_msgs::image_encodings::RGB8;
			cvImageBr.image = croppedImage;
			cvImageBr.toImageMsg(face.source_img);
		}

		return face;
	}
};

#endif

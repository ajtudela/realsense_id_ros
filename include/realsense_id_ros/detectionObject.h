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

struct DetectionObject{
	void sanitizeSize(const size_t& imageWidth, const size_t& imageHeight){
		x = (x < 0) ? 0 : x;
		y = (y < 0) ? 0 : y;
		width = ((x + width) > imageWidth) ? (imageWidth - x) : width;
		height = ((y + height) > imageHeight) ? (imageHeight - y) : height;
	}

	size_t x, y, width, height;
	float confidence;
	std::string id;
	bool hasMask;
};

#endif

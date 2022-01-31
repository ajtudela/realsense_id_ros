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
	int x, y, width, height;
	float confidence;
	std::string id;
	bool hasMask;
};

#endif

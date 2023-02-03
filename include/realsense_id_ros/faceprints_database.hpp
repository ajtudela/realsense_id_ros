/*
 * FACEPRINTS DATABASE STRUCT
 *
 * Copyright (c) 2022-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#ifndef REALSENSE_ID_ROS__FACEPRINTS_DATABASE_HPP_
#define REALSENSE_ID_ROS__FACEPRINTS_DATABASE_HPP_

// C++
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <cassert>

// ROS
#include "rclcpp/rclcpp.hpp"

// RealSense ID
#include <RealSenseID/Faceprints.h>

#include "realsense_id_ros/xpjson.hpp"

struct FaceprintsDatabase{
	std::map<std::string, RealSenseID::Faceprints> data;

	/* Remove one user from the database */
	bool remove_user(const std::string & user_id){
		return data.erase(user_id);
	}

	/* Remove all users from the database */
	void remove_all_users(){
		data.clear();
	}

	/* Get the number of users */
	int get_number_of_users(){
		return data.size();
	}

	/* Get vector of users */
	std::vector<std::string> get_users(){
		std::vector<std::string> users;
		for (const auto& iter: data){
			users.push_back(iter.first);
		}
		return users;
	}

	/* Convert the database to JSON::Value */
	JSON::Value to_json(){
		JSON::Value json_root;

		/* Create the array of users */
		JSON::Array& db = json_root["db"].a();

		/* Loop through the users */
		for (const auto &pair: data){
			std::string user_id = pair.first;
			RealSenseID::Faceprints fprints = pair.second;

			// Create an user
			JSON::Value user;
			// Create id
			JSON::Object& id = user.o();
			id["userID"].s() = user_id;
			// Create faceprints
			JSON::Object& faceprints = user["faceprints"].o();
			JSON::Array& reserved = faceprints["reserved"].a();
			for (const auto &r: fprints.data.reserved){
				reserved.push_back(r);
			}
			faceprints["version"] = fprints.data.version;
			faceprints["featuresType"] = fprints.data.featuresType;
			faceprints["flags"] = fprints.data.flags;
			JSON::Array& adaptive_descriptor_without_mask = faceprints["adaptiveDescriptorWithoutMask"].a();
			for (const auto &r: fprints.data.adaptiveDescriptorWithoutMask){
				adaptive_descriptor_without_mask.push_back(r);
			}
			JSON::Array& adaptive_descriptor_with_mask = faceprints["adaptiveDescriptorWithMask"].a();
			for (const auto &r: fprints.data.adaptiveDescriptorWithMask){
				adaptive_descriptor_with_mask.push_back(r);
			}
			JSON::Array& enrollment_descriptor = faceprints["enrollmentDescriptor"].a();
			for (const auto &r: fprints.data.enrollmentDescriptor){
				enrollment_descriptor.push_back(r);
			}
			// Push the user to db
			db.push_back(user);
		}

		return json_root;
	};

	/* Convert the database to std::string */
	std::string to_string(){
		std::string json_str;
		JSON::Value json_root = to_json();
		json_root.write(json_str);
		return json_str;
	}

	/* Convert the database from a JSON::Value */
	void from_json(JSON::Value json_root){
		/* Create the array of users */
		JSON::Array& db = json_root["db"].a();

		/* Extract the users */
		for (size_t u = 0; u < db.size(); ++u){
			std::string user_id;
			RealSenseID::Faceprints fprints;

			// Create an user
			JSON::Value user = db[u];
			// Create id
			JSON::Object& id = user.o();
			user_id = id["userID"].s();
			// Create faceprints
			JSON::Object& faceprints = user["faceprints"].o();
			JSON::Array& reserved = faceprints["reserved"].a();
			int count = 0;
			for (JSON::Array::const_iterator it=reserved.begin(); it!=reserved.end(); ++it){
				fprints.data.reserved[count++] = it->i();
			}
			fprints.data.version = faceprints["version"].i();
			fprints.data.featuresType = faceprints["featuresType"].i();
			fprints.data.flags = faceprints["flags"].i();
			JSON::Array& adaptive_descriptor_without_mask = faceprints["adaptiveDescriptorWithoutMask"].a();
			count = 0;
			for (JSON::Array::const_iterator it=adaptive_descriptor_without_mask.begin(); it!=adaptive_descriptor_without_mask.end(); ++it){
				fprints.data.adaptiveDescriptorWithoutMask[count++] = it->i();
			}
			JSON::Array& adaptive_descriptor_with_mask = faceprints["adaptiveDescriptorWithMask"].a();
			count = 0;
			for (JSON::Array::const_iterator it=adaptive_descriptor_with_mask.begin(); it!=adaptive_descriptor_with_mask.end(); ++it){
				fprints.data.adaptiveDescriptorWithMask[count++] = it->i();
			}
			JSON::Array& enrollment_descriptor = faceprints["enrollmentDescriptor"].a();
			count = 0;
			for (JSON::Array::const_iterator it=enrollment_descriptor.begin(); it!=enrollment_descriptor.end(); ++it){
				fprints.data.enrollmentDescriptor[count++] = it->i();
			}
			// Push the user to db
			data[user_id] = fprints;
		}
	}

	/* Convert the database from a std::string */
	void from_string(std::string json_str){
		JSON::Value json_root;
		size_t ret = json_root.read(json_str);
		assert(ret == json_str.length());
		from_json(json_root);
	}

	/* Load the database from a file */
	void load_db_from_file(std::string db_filepath){
		std::ifstream db_file(db_filepath);
		if (!db_file.is_open()){
			RCLCPP_ERROR(rclcpp::get_logger("RealSenseID"), "Error opening file %s", db_filepath.c_str());
			exit(1);
		}

		std::string json_str;
		db_file >> json_str;
		from_string(json_str);
	}

	/* Save the database to a file */
	void save_db_to_file(std::string db_filepath){
		std::ofstream db_file(db_filepath);
		if (!db_file){
			RCLCPP_ERROR(rclcpp::get_logger("RealSenseID"), "Error opening file %s", db_filepath.c_str());
			exit(1);
		}

		db_file << to_string();
		db_file.close();
	}
};

#endif  // REALSENSE_ID_ROS__FACEPRINTS_DATABASE_HPP_

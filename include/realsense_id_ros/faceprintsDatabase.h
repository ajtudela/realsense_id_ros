/*
 * FACEPRINTS DATABASE STRUCT
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#ifndef FACEPRINTS_DATABASE_H
#define FACEPRINTS_DATABASE_H

// C++
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <cassert>

#include <RealSenseID/Faceprints.h>
#include "realsense_id_ros/xpjson.hpp"

struct FaceprintsDatabase{
	std::map<std::string, RealSenseID::Faceprints> data;

	/* Remove one user from the database */
	bool removeUser(std::string userId){
		return data.erase(userId);
	}

	/* Remove all users from the database */
	void removeAllUsers(){
		data.clear();
	}

	/* Get the number of users */
	int getNumberOfUsers(){
		return data.size();
	}

	/* Get vector of users */
	std::vector<std::string> getUsers(){
		std::vector<std::string> users;
		for (const auto& iter: data){
			users.push_back(iter.first);
		}
		return users;
	}

	/* Convert the database to JSON::Value */
	JSON::Value toJSON(){
		JSON::Value jsonRoot;

		/* Create the array of users */
		JSON::Array& db = jsonRoot["db"].a();

		/* Loop through the users */
		for (const auto &pair: data){
			std::string userId = pair.first;
			RealSenseID::Faceprints fprints = pair.second;

			// Create an user
			JSON::Value user;
			// Create id
			JSON::Object& id = user.o();
			id["userID"].s() = userId;
			// Create faceprints
			JSON::Object& faceprints = user["faceprints"].o();
			JSON::Array& reserved = faceprints["reserved"].a();
			for (const auto &r: fprints.data.reserved){
				reserved.push_back(r);
			}
			faceprints["version"] = fprints.data.version;
			faceprints["featuresType"] = fprints.data.featuresType;
			faceprints["flags"] = fprints.data.flags;
			JSON::Array& adaptiveDescriptorWithoutMask = faceprints["adaptiveDescriptorWithoutMask"].a();
			for (const auto &r: fprints.data.adaptiveDescriptorWithoutMask){
				adaptiveDescriptorWithoutMask.push_back(r);
			}
			JSON::Array& adaptiveDescriptorWithMask = faceprints["adaptiveDescriptorWithMask"].a();
			for (const auto &r: fprints.data.adaptiveDescriptorWithMask){
				adaptiveDescriptorWithMask.push_back(r);
			}
			JSON::Array& enrollmentDescriptor = faceprints["enrollmentDescriptor"].a();
			for (const auto &r: fprints.data.enrollmentDescriptor){
				enrollmentDescriptor.push_back(r);
			}
			// Push the user to db
			db.push_back(user);
		}

		return jsonRoot;
	};

	/* Convert the database to std::string */
	std::string toString(){
		std::string jsonStr;
		JSON::Value jsonRoot = toJSON();
		jsonRoot.write(jsonStr);
		return jsonStr;
	}

	/* Convert the database from a JSON::Value */
	void fromJSON(JSON::Value jsonRoot){
		/* Create the array of users */
		JSON::Array& db = jsonRoot["db"].a();

		/* Extract the users */
		for (size_t u = 0; u < db.size(); ++u){
			std::string userId;
			RealSenseID::Faceprints fprints;

			// Create an user
			JSON::Value user = db[u];
			// Create id
			JSON::Object& id = user.o();
			userId = id["userID"].s();
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
			JSON::Array& adaptiveDescriptorWithoutMask = faceprints["adaptiveDescriptorWithoutMask"].a();
			count = 0;
			for (JSON::Array::const_iterator it=adaptiveDescriptorWithoutMask.begin(); it!=adaptiveDescriptorWithoutMask.end(); ++it){
				fprints.data.adaptiveDescriptorWithoutMask[count++] = it->i();
			}
			JSON::Array& adaptiveDescriptorWithMask = faceprints["adaptiveDescriptorWithMask"].a();
			count = 0;
			for (JSON::Array::const_iterator it=adaptiveDescriptorWithMask.begin(); it!=adaptiveDescriptorWithMask.end(); ++it){
				fprints.data.adaptiveDescriptorWithMask[count++] = it->i();
			}
			JSON::Array& enrollmentDescriptor = faceprints["enrollmentDescriptor"].a();
			count = 0;
			for (JSON::Array::const_iterator it=enrollmentDescriptor.begin(); it!=enrollmentDescriptor.end(); ++it){
				fprints.data.enrollmentDescriptor[count++] = it->i();
			}
			// Push the user to db
			data[userId] = fprints;
		}
	}

	/* Convert the database from a std::string */
	void fromString(std::string jsonStr){
		JSON::Value jsonRoot;
		size_t ret = jsonRoot.read(jsonStr);
		assert(ret == jsonStr.length());
		fromJSON(jsonRoot);
	}

	/* Load the database from a file */
	void loadDbFromFile(std::string dbFilepath){
		std::ifstream dbFile(dbFilepath);
		if (!dbFile.is_open()){
			ROS_ERROR_STREAM("[RealSense ID]: Error opening file " << dbFilepath);
			exit(1);
		}

		std::string jsonStr;
		dbFile >> jsonStr;
		fromString(jsonStr);
	}

	/* Save the database to a file */
	void saveDbToFile(std::string dbFilepath){
		std::ofstream dbFile(dbFilepath);
		if (!dbFile){
			ROS_ERROR_STREAM("[RealSense ID]: Error opening file " << dbFilepath);
			exit(1);
		}

		dbFile << toString();
		dbFile.close();
	}
};

#endif

/*
 * REALSENSE SERVER CALLBACKS CLASSES
 *
 * Copyright (c) 2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#ifndef REALSENSE_SERVER_CALLBACKS_H
#define REALSENSE_SERVER_CALLBACKS_H

// C++
#include <string>
#include <map>
#include <memory>

// RealSense ID
#include <RealSenseID/AuthFaceprintsExtractionCallback.h>
#include <RealSenseID/AuthenticateStatus.h>
#include <RealSenseID/EnrollFaceprintsExtractionCallback.h>
#include <RealSenseID/EnrollStatus.h>
#include <RealSenseID/FacePose.h>

// ROS
#include <ros/ros.h>

#include "detectionObject.h"

static std::map<std::string, RealSenseID::Faceprints> faceprintsDB;

class RSAuthFaceprintsCallback: public RealSenseID::AuthFaceprintsExtractionCallback{
	public:
		RSAuthFaceprintsCallback(RealSenseID::FaceAuthenticator* authenticator): authenticator_(authenticator){}

		void clear(){
			detections_.clear();
			results_ = 0;
		};

		void OnResult(const RealSenseID::AuthenticateStatus status, const RealSenseID::ExtractedFaceprints* faceprints) override{
			ROS_DEBUG_STREAM("[RealSense ID]: Authenticate " << status);

			if(status != RealSenseID::AuthenticateStatus::Success){
				ROS_DEBUG_STREAM("[RealSense ID]: ExtractFaceprints failed with status" << status);
				return;
			}

			RealSenseID::MatchElement scannedFaceprint;
			scannedFaceprint.data.version = faceprints->data.version;
			scannedFaceprint.data.featuresType = faceprints->data.featuresType;

			int32_t vecFlags = (int32_t)faceprints->data.featuresVector[RSID_INDEX_IN_FEATURES_VECTOR_TO_FLAGS];
			int32_t opFlags = RealSenseID::FaOperationFlagsEnum::OpFlagAuthWithoutMask;

			if(vecFlags == RealSenseID::FaVectorFlagsEnum::VecFlagValidWithMask){
				opFlags = RealSenseID::FaOperationFlagsEnum::OpFlagAuthWithMask;
			}
			scannedFaceprint.data.flags = opFlags;
			static_assert(sizeof(scannedFaceprint.data.featuresVector) == sizeof(faceprints->data.featuresVector), "faceprints without mask sizes does not match");
			::memcpy(scannedFaceprint.data.featuresVector, faceprints->data.featuresVector, sizeof(faceprints->data.featuresVector));

			// Try to match the resulting faceprint to one of the faceprints stored in the db
			RealSenseID::Faceprints updatedFaceprint;

			ROS_DEBUG_STREAM("[RealSense ID]: Searching " << faceprintsDB.size() << " faceprints");

			int saveMaxScore = -1;
			int winningIndex = -1;
			std::string winningIdStr = "";
			RealSenseID::MatchResultHost winningMatchResult;
			RealSenseID::Faceprints winningUpdatedFaceprints;

			// Use High by default.
			// Should be taken from DeviceConfig.
			RealSenseID::ThresholdsConfidenceEnum matcherConfidenceLevel = RealSenseID::ThresholdsConfidenceEnum::ThresholdsConfidenceLevel_High;

			int usersIndex = 0;

			for(auto& iter: faceprintsDB){
				auto& userId = iter.first;
				auto& existingFaceprint = iter.second;  // faceprints at the DB
				auto& updatedFaceprint = existingFaceprint; // updated faceprints

				auto match = authenticator_->MatchFaceprints(scannedFaceprint, existingFaceprint, existingFaceprint, matcherConfidenceLevel);

				int currentScore = (int)match.score;

				// Save the best winner that matched.
				if (match.success){
					if(currentScore > saveMaxScore){
						saveMaxScore = currentScore;
						winningMatchResult = match;
						winningIndex = usersIndex;
						winningIdStr = userId;
						winningUpdatedFaceprints = updatedFaceprint;
					}
				}
				usersIndex++;
			}

			// We have a winner so declare success!
			if(winningIndex >= 0){
				ROS_DEBUG_STREAM("[RealSense ID]: Match success. user_id: " << winningIdStr);
				// Apply adaptive-update on the db.
				if(winningMatchResult.should_update){
					// Apply adaptive update
					faceprintsDB[winningIdStr] = winningUpdatedFaceprints;
					ROS_DEBUG_STREAM("[RealSense ID]: DB adaptive apdate applied to user = " << winningIdStr << ".");
				}
			}else{ // no winner, declare authentication failed!
				ROS_DEBUG_STREAM("[RealSense ID: Forbidden (no faceprint matched)");
			}

			// Check results and add them to objects
			if(faces_.size() > results_){
				auto& face = faces_[results_];

				// Create new detection
				DetectionObject newDetection;
				newDetection.x = face.x;
				newDetection.y = face.y;
				newDetection.width = face.w;
				newDetection.height = face.h;
				newDetection.id = winningIdStr;
				newDetection.confidence = 0.0;
				detections_.push_back(newDetection);

				ROS_DEBUG("[RealSense ID]: Detected face %u,%u %ux%u", face.x, face.y, face.w, face.h);
			}
			results_++;
		}

		void OnHint(const RealSenseID::AuthenticateStatus hint) override{
			ROS_DEBUG_STREAM("[RealSense ID]: Hint " << hint);
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
		RealSenseID::FaceAuthenticator* authenticator_;
		std::vector<DetectionObject> detections_;
		std::vector<RealSenseID::FaceRect> faces_;
		size_t results_ = 0;
		unsigned int ts_ = 0;
};

class RSEnrollFaceprintsCallback: public RealSenseID::EnrollFaceprintsExtractionCallback{
	public:
		RSEnrollFaceprintsCallback(const char* user_id): userId_(user_id){}

		void clear(){
			detections_.clear();
			results_ = 0;
		};

		void OnResult(const RealSenseID::EnrollStatus status, const RealSenseID::ExtractedFaceprints* faceprints) override{
			ROS_DEBUG_STREAM("[RealSense ID]: Result " << status);
			if(status == RealSenseID::EnrollStatus::Success){
				faceprintsDB[userId_].data.version = faceprints->data.version;
				faceprintsDB[userId_].data.flags = faceprints->data.flags;
				faceprintsDB[userId_].data.featuresType = faceprints->data.featuresType;

				// Set the full data for the enrolled object:
				size_t copySize = sizeof(faceprints->data.featuresVector);

				static_assert(sizeof(faceprintsDB[userId_].data.adaptiveDescriptorWithoutMask) == sizeof(faceprints->data.featuresVector), "faceprints sizes does not match");
				::memcpy(faceprintsDB[userId_].data.adaptiveDescriptorWithoutMask, faceprints->data.featuresVector, copySize);

				static_assert(sizeof(faceprintsDB[userId_].data.enrollmentDescriptor) == sizeof(faceprints->data.featuresVector), "faceprints sizes does not match");
				::memcpy(faceprintsDB[userId_].data.enrollmentDescriptor, faceprints->data.featuresVector, copySize);

				// Mark the withMask vector as not-set because its not yet set!
				faceprintsDB[userId_].data.adaptiveDescriptorWithMask[RSID_INDEX_IN_FEATURES_VECTOR_TO_FLAGS] = RealSenseID::FaVectorFlagsEnum::VecFlagNotSet;

				// Check results and add them to objects
				if(faces_.size() > results_){
					auto& face = faces_[results_];

					// Create new detection
					DetectionObject newDetection;
					newDetection.x = face.x;
					newDetection.y = face.y;
					newDetection.width = face.w;
					newDetection.height = face.h;
					newDetection.confidence = 0.0;
					detections_.push_back(newDetection);

					ROS_DEBUG("[RealSense ID]: Detected face %u,%u %ux%u", face.x, face.y, face.w, face.h);
				}
				results_++;
			}
		}

		void OnProgress(const RealSenseID::FacePose pose) override{
			ROS_DEBUG_STREAM("[RealSense ID]: Progress " << pose);
		}

		void OnHint(const RealSenseID::EnrollStatus hint) override{
			ROS_DEBUG_STREAM("[RealSense ID]: Hint " << hint);
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
		std::string userId_;
		std::vector<DetectionObject> detections_;
		std::vector<RealSenseID::FaceRect> faces_;
		size_t results_ = 0;
		unsigned int ts_ = 0;
};


#endif

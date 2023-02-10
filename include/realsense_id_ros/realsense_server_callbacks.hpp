/*
 * REALSENSE SERVER CALLBACKS CLASSES
 *
 * Copyright (c) 2021-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of realsense_id_ros project.
 * 
 * All rights reserved.
 *
 */

#ifndef REALSENSE_ID_ROS__REALSENSE_SERVER_CALLBACKS_HPP_
#define REALSENSE_ID_ROS__REALSENSE_SERVER_CALLBACKS_HPP_

// C++
#include <fstream>
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
#include <rclcpp/rclcpp.hpp>

#include "realsense_id_ros/detection_object.hpp"

const int RSID_MIN_POSSIBLE_SCORE = 0;
const int RSID_MAX_POSSIBLE_SCORE = 4096;

typedef std::map<std::string, RealSenseID::Faceprints> RSFaceprintsDatabase;

class RSAuthFaceprintsCallback: public RealSenseID::AuthFaceprintsExtractionCallback{
	public:
		RSAuthFaceprintsCallback(){};

		RSAuthFaceprintsCallback(RealSenseID::FaceAuthenticator* authenticator, RSFaceprintsDatabase faceprintsDB): authenticator_(authenticator), faceprints_db_(faceprintsDB){}

		void clear(){
			detections_.clear();
			results_ = 0;
		};

		void OnResult(const RealSenseID::AuthenticateStatus status, const RealSenseID::ExtractedFaceprints* faceprints) override{
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Authenticate " << status);

			if (status != RealSenseID::AuthenticateStatus::Success){
				RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "ExtractFaceprints failed with status" << status);
				return;
			}

			RealSenseID::MatchElement scanned_faceprint;
			scanned_faceprint.data.version = faceprints->data.version;
			scanned_faceprint.data.featuresType = faceprints->data.featuresType;

			int32_t vec_flags = (int32_t)faceprints->data.featuresVector[RSID_INDEX_IN_FEATURES_VECTOR_TO_FLAGS];
			int32_t op_flags = RealSenseID::FaOperationFlagsEnum::OpFlagAuthWithoutMask;

			if (vec_flags == RealSenseID::FaVectorFlagsEnum::VecFlagValidWithMask){
				op_flags = RealSenseID::FaOperationFlagsEnum::OpFlagAuthWithMask;
			}
			scanned_faceprint.data.flags = op_flags;
			static_assert(sizeof(scanned_faceprint.data.featuresVector) == sizeof(faceprints->data.featuresVector), "faceprints without mask sizes does not match");
			::memcpy(scanned_faceprint.data.featuresVector, faceprints->data.featuresVector, sizeof(faceprints->data.featuresVector));

			// Try to match the resulting faceprint to one of the faceprints stored in the db
			RealSenseID::Faceprints update_faceprint;

			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Searching " << faceprints_db_.size() << " faceprints");

			int save_max_score = -1;
			int winning_index = -1;
			std::string winning_id_str = "";
			RealSenseID::MatchResultHost winning_match_result;
			RealSenseID::Faceprints winning_update_faceprints;

			// Use High by default.
			// Should be taken from DeviceConfig.
			RealSenseID::ThresholdsConfidenceEnum matcher_confidence_level = RealSenseID::ThresholdsConfidenceEnum::ThresholdsConfidenceLevel_High;

			int users_index = 0;

			for (auto& iter: faceprints_db_){
				auto& user_id = iter.first;
				auto& existing_faceprint = iter.second;  // faceprints at the DB
				auto& update_faceprint = existing_faceprint; // updated faceprints

				auto match = authenticator_->MatchFaceprints(scanned_faceprint, existing_faceprint, existing_faceprint, matcher_confidence_level);

				int current_score = (int)match.score;

				// Save the best winner that matched.
				if (match.success){
					if (current_score > save_max_score){
						save_max_score = current_score;
						winning_match_result = match;
						winning_index = users_index;
						winning_id_str = user_id;
						winning_update_faceprints = update_faceprint;
					}
				}
				users_index++;
			}

			// We have a winner so declare success!
			if (winning_index >= 0){
				RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Match success. user_id: " << winning_id_str);
				// Apply adaptive-update on the db.
				if (winning_match_result.should_update){
					// Apply adaptive update
					faceprints_db_[winning_id_str] = winning_update_faceprints;
					RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "DB adaptive apdate applied to user = " << winning_id_str << ".");
				}
			}else{ // no winner, declare authentication failed!
				RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Forbidden (no faceprint matched)");
			}

			// Check results and add them to objects
			if (faces_.size() > results_){
				auto& face = faces_[results_];

				// Create new detection
				DetectionObject new_detection;
				new_detection.x = face.x;
				new_detection.y = face.y;
				new_detection.width = face.w;
				new_detection.height = face.h;
				new_detection.id = winning_id_str;
				new_detection.confidence = static_cast<float>(save_max_score) / RSID_MAX_POSSIBLE_SCORE;
				new_detection.has_mask = (vec_flags == RealSenseID::FaVectorFlagsEnum::VecFlagValidWithMask) ? true : false;
				detections_.push_back(new_detection);

				RCLCPP_DEBUG(rclcpp::get_logger("RealSenseID"), "Detected face %u,%u %ux%u", 
							face.x, face.y, face.w, face.h);
			}
			results_++;
		}

		void OnHint(const RealSenseID::AuthenticateStatus hint) override{
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Hint " << hint);
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

		void setAuthenticator(RealSenseID::FaceAuthenticator* authenticator){
			authenticator_ = authenticator;
		}

		void setFaceprintsDatabase(RSFaceprintsDatabase faceprintsDB){
			faceprints_db_ = faceprintsDB;
		}

	private:
		RealSenseID::FaceAuthenticator* authenticator_;
		RSFaceprintsDatabase faceprints_db_;
		std::vector<DetectionObject> detections_;
		std::vector<RealSenseID::FaceRect> faces_;
		size_t results_ = 0;
		unsigned int ts_ = 0;
};

class RSEnrollFaceprintsCallback: public RealSenseID::EnrollFaceprintsExtractionCallback{
	public:
		RSEnrollFaceprintsCallback(const char* user_id, RSFaceprintsDatabase faceprintsDB): user_id_(user_id), faceprints_db_(faceprintsDB){}

		void clear(){
			detections_.clear();
			results_ = 0;
		};

		void OnResult(const RealSenseID::EnrollStatus status, const RealSenseID::ExtractedFaceprints* faceprints) override{
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Result " << status);

			if (status == RealSenseID::EnrollStatus::Success){
				faceprints_db_[user_id_].data.version = faceprints->data.version;
				faceprints_db_[user_id_].data.flags = faceprints->data.flags;
				faceprints_db_[user_id_].data.featuresType = faceprints->data.featuresType;

				// Set the full data for the enrolled object:
				size_t copySize = sizeof(faceprints->data.featuresVector);

				static_assert(sizeof(faceprints_db_[user_id_].data.adaptiveDescriptorWithoutMask) == sizeof(faceprints->data.featuresVector), "faceprints sizes does not match");
				::memcpy(faceprints_db_[user_id_].data.adaptiveDescriptorWithoutMask, faceprints->data.featuresVector, copySize);

				static_assert(sizeof(faceprints_db_[user_id_].data.enrollmentDescriptor) == sizeof(faceprints->data.featuresVector), "faceprints sizes does not match");
				::memcpy(faceprints_db_[user_id_].data.enrollmentDescriptor, faceprints->data.featuresVector, copySize);

				// Mark the withMask vector as not-set because its not yet set!
				faceprints_db_[user_id_].data.adaptiveDescriptorWithMask[RSID_INDEX_IN_FEATURES_VECTOR_TO_FLAGS] = RealSenseID::FaVectorFlagsEnum::VecFlagNotSet;

				// Check results and add them to objects
				if (faces_.size() > results_){
					auto& face = faces_[results_];

					// Create new detection
					DetectionObject new_detection;
					new_detection.x = face.x;
					new_detection.y = face.y;
					new_detection.width = face.w;
					new_detection.height = face.h;
					new_detection.has_mask = false;
					new_detection.confidence = 1.0;
					detections_.push_back(new_detection);

					RCLCPP_DEBUG(rclcpp::get_logger("RealSenseID"), "Detected face %u,%u %ux%u", face.x, face.y, face.w, face.h);
				}
				results_++;
			}
		}

		void OnProgress(const RealSenseID::FacePose pose) override{
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Progress " << pose);
		}

		void OnHint(const RealSenseID::EnrollStatus hint) override{
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RealSenseID"), "Hint " << hint);
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

		RSFaceprintsDatabase getDatabase(){
			return faceprints_db_;
		}

	private:
		std::string user_id_;
		RSFaceprintsDatabase faceprints_db_;
		std::vector<DetectionObject> detections_;
		std::vector<RealSenseID::FaceRect> faces_;
		size_t results_ = 0;
		unsigned int ts_ = 0;
};


#endif  // REALSENSE_ID_ROS__REALSENSE_SERVER_CALLBACKS_HPP_

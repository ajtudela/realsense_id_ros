^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense_id_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (20-01-2022)
------------------
* Add device info service.
* Update score for server mode.
* Fix OpenCv assert in preview image.

1.1.0 (17-01-2022)
------------------
* Add restore to defaults in dynamic reconfigure.
* Publish id when enrollment.
* Add status to enroll.
* Publish array of faces when loop mode is on.
* Change confidences to 100.
* Add authentication_loop to launch file.

1.0.12 (06-09-2021)
------------------
* Fix dates in changelog ( •_•)
* Update compatibility with Intel RealSenseID SDK v0.25.0.

1.0.11 (02-08-2021)
------------------
* Added a publisher image.
* Added authenticate loop mode as a dynamic reconfigure parameter.
* Remove preview timestamp.

1.0.10 (01-08-2021)
------------------
* Move callbacks to local scope.
* Add preview to enroll.
* Rearrange headers files.
* Added console log in remove user. 
* Added support for server mode.

1.0.9 (29-07-2021)
------------------
* Add timestamps.
* Fix noDetected face array issue.
* Added internal struct.
* Change Face.msg with vision_msgs BoundingBox;
* Remove unnecessary Rect.msg.

1.0.8 (29-07-2021)
------------------
* Preview image with the faces.
* Update cfg with dump mode.

1.0.7 (29-07-2021)
------------------
* Rename node to "realsense_id_ros" instead of "realsense_id_ros_node"
* Update cfg and cpp files to Intel RealSenseID SDK v0.23.0.
* Update README with the changes. 
* Added version to match en CMakelists.txt.

1.0.6 (09-07-2021)
------------------
* Added dependencies in /opt/ instead of $HOME by install RealSenseID SDK with latest version of Cmake.

1.0.5 (07-07-2021)
------------------
* Added dynamic reconfigure server to change device onboard configuration.
* Added multiple faces authentication.

1.0.0 (06-07-2021)
------------------
* Initial release.
* Create README.md.
* Create CHANGELOG.rst.
* Create LICENSE.
* Added RealSense callbacks header.
* Added Rect.msg and Face.msg messages.
* Added Authenticate.srv, Enroll.srv, QueryUsersId.srv, RemoveUser.srv.
* Added authenticate user, enroll new user, query the users in the db, remove a specific user and remove all user ROS services using message described above. 
* Added RealSenseIDROS class (.h and .cpp files).
* Added realsense_id_ros_node.
* Contributors: Alberto Tudela

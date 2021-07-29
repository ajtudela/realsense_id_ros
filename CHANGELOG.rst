^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense_id_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.0.8 (29-07-2020)
------------------
* Preview image with the faces.
* Update cfg with dump mode.

1.0.7 (29-07-2020)
------------------
* Rename node to "realsense_id_ros" instead of "realsense_id_ros_node"
* Update cfg and cpp files to Intel RealSenseID SDK v0.23.0.
* Update README with the changes. 
* Added version to mathc en CMakelists.txt.

1.0.6 (09-07-2020)
------------------
* Added dependencies in /opt/ instead of $HOME by install RealSenseID SDK with latest version of Cmake.

1.0.5 (07-07-2020)
------------------
* Added dynamic reconfigure server to change device onboard configuration.
* Added multiple faces authentication.

1.0.0 (06-07-2020)
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

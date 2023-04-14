# realsense_id_ros

![ROS2](https://img.shields.io/badge/ros2-galactic-purple?logo=ros&logoColor=white)
![License](https://img.shields.io/badge/license-Apache%202-blue)

## Overview

This package enables the use of [Intel RealSense ID] (F455 and F450) cameras with ROS. The Intel RealSense ID camera is a facial authentication on-device solution.

The package exposes the camera as a ROS node, allowing users to configure it for authentication and enrollment in either device mode (inside the camera) or server mode (using a faceprint database). Additionally, it publishes the preview image.

In both device and server modes, authentication and enrollment are available as ROS services. Other services that can be accessed include user removal and querying the list of IDs.

Furthermore, all camera parameters can be changed online using the device's onboard configuration.

**Keywords:** ROS2, Intel RealSense ID, face recognition

### License

The source code is released under a [Apache license 2.0](LICENSE).

**Author: Alberto Tudela<br />**

The realsense_id_ros package has been tested under [ROS2] Galactic on [Ubuntu] 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/galactic/) (middleware for robotics),
- [Intel RealSense ID SDK v0.25](https://github.com/IntelRealSense/RealSenseID)
- [face_msgs](https://github.com/grupo-avispa/face_msgs) (Messages and services for face recognition)

Build the Intel RealSense ID SDK as follows:

```console
$ cd $HOME
$ git clone https://github.com/grupo-avispa/RealSenseID.git
$ cd RealSenseID
$ mkdir build && cd build
$ cmake .. -DRSID_PREVIEW=1 -DRSID_DEBUG_CONSOLE=OFF -DRSID_INSTALL=ON
$ make -j
$ sudo make install
```

After, Intel RealsenseID permissions script from root directory:
```console
./scripts/setup_udev_rules.sh
```

#### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using

	cd colcon_workspace/src
	git clone https://github.com/ajtudela/realsense_id_ros.git -b galactic
	cd ../
	rosdep install -i --from-path src --rosdistro galactic -y
	colcon build --symlink-install

## Usage

To start the camera node in ROS:

	ros2 run realsense_id_ros realsense_id_ros_node

## Nodes

### realsense_id_ros_node

Camera node to perform facial recognition.

#### Published Topics

* **`faces`** ([face_msgs/FaceArray])

	Array with the detected faces.

* **`camera_info`** ([sensor_msgs/CameraInfo])

	Information about the camera intrinsic parameters.

* **`image_raw`** ([sensor_msgs/Image])

	Image with the bounding boxes surrounding the detected faces.

#### Services

* **`authenticate`** ([face_msgs/Authenticate])

	Perform one authentication on the device or on the server. Returns an array of faces with users id and an image of the faces. For example, you can trigger the computation from the console with

		ros2 interface call /realsense_id_ros_node/authenticate

* **`device_info`** ([face_msgs/DeviceInfo])

	Get information of the device. on the server. For example, you can trigger the computation from the console with

		ros2 interface call /realsense_id_ros_node/device_info

* **`enroll`** ([face_msgs/Enroll])

	Perform one enrollment for one new user on the device or on the server. Returns an array of faces with image of the faces. For example, you can trigger the computation from the console with

		ros2 interface call /realsense_id_ros_node/enroll

* **`remove_user`** ([face_msgs/RemoveUser])

	Remove an user from the device or from the server database. For example, you can trigger the computation from the console with

		ros2 interface call /realsense_id_ros_node/remove_user

* **`remove_all_users`** ([face_msgs/RemoveAllUsers])

	Remove all users from the device or from the server database. For example, you can trigger the computation from the console with

		ros2 interface call /realsense_id_ros_node/remove_all

* **`query_users_id`** ([face_msgs/QueryUsersId])

	Query the ids of the users. Returns the number of users and an array of ids. For example, you can trigger the computation from the console with

		ros2 interface call /realsense_id_ros_node/query_users_id

#### Parameters

* **`serial_port`** (string, default: "/dev/ttyACM0")

	Will attach to the device with the given serial port.

* **`server_mode`** (bool, default: "false")

	Option to manage a faceprints database on the host or the server.

* **`database`** (string, default: "")

	Path to the database folder.

* **`camera_rotation`** (int, default: "0")

	Enable the algorithm to work with a rotated device.

* **`security_level`** (string, default: "medium")

	Set security level to 'high' to allow no mask suport or to level 'medium' to support masks.

* **`algo_flow`** (string, default: "all")

	Algorithms which will be used during authentication: 'all', 'detection', 'recognition' and 'spoof'.

* **`face_selection_policy`** (string, default: "all")

	Face selection policy to run authentication on 'all' (up to 5) detected faces vs 'single' (closest) face.

* **`dump_mode`** (string, default: "none")

	Set mode for dump image: 'none', 'cropped' or 'fullframe'.

* **`matcher_confidence_level`** (string, default: "medium")

	Used in the matcher during authentication, each level means a different set of threshold is used.


## Future work
- [x] Use global dependencies.
- [x] Preview snapshot of the faces.
- [x] Multiple faces.
- [x] Host mode.
- [x] Load and backup of faceprints database in a JSON file.
- [ ] Pair device.
- [ ] Secure mode.
- [x] Option to use authentication loop with published topics.
- [x] Dynamic reconfigure server.
- [ ] Extract features from RGB image.
- [x] Add mutex to the captured image.
- [ ] Convert nodes to LifeCycleNodes.

[Intel RealSense ID]: https://www.intelrealsense.com/facial-authentication/
[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/galactic/
[sensor_msgs/CameraInfo]: http://docs.ros2.org/galactic/api/sensor_msgs/msg/CameraInfo.html
[sensor_msgs/Image]: http://docs.ros2.org/galactic/api/sensor_msgs/msg/Image.html
[face_msgs/FaceArray]: https://github.com/grupo-avispa/face_msgs/blob/-/msg/FaceArray.msg
[face_msgs/Authenticate]: https://github.com/grupo-avispa/face_msgs/blob/-/srv/Authenticate.srv
[face_msgs/DeviceInfo]: https://github.com/grupo-avispa/face_msgs/blob/-/srv/DeviceInfo.srv
[face_msgs/Enroll]: https://github.com/grupo-avispa/face_msgs/blob/-/srv/Enroll.srv
[face_msgs/RemoveUser]: https://github.com/grupo-avispa/face_msgs/blob/-/srv/RemoveUser.srv
[face_msgs/RemoveAllUsers]: https://github.com/grupo-avispa/face_msgs/blob/-/srv/RemoveAllUsers.srv
[face_msgs/QueryUsersId]: https://github.com/grupo-avispa/face_msgs/blob/-/srv/QueryUsersId.srv
# realsense_id_ros

## Overview

This is a package for using [Intel RealSense ID] (F455 and F450) cameras with ROS. Intel RealSense ID is a facial authentication on-device solution.

It exposes the camera as a ROS node and allows the user to perform authentication and enrollment as ROS services. Other services exposed are: removement of users and query the list of ids.

Includes a dynamic reconfigure server paarameter to change the device onboard configuration. 

**Keywords:** ROS, Intel RealSense ID, face recognition

### License

The source code is released under a [Apache license 2.0](LICENSE).

**Author: Alberto Tudela<br />**

The realsense_id_ros package has been tested under [ROS] Melodic on [Ubuntu] 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Intel RealSense ID SDK](https://github.com/IntelRealSense/RealSenseID) 

Build the Intel RealSense ID SDK as follows:

```console
$ cd $HOME
$ git clone https://github.com/IntelRealSense/RealSenseID
$ mkdir build && cd build
$ cmake .. -DRSID_PREVIEW=1 -DRSID_DEBUG_CONSOLE=OFF
$ make -j
```
#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

```console
$ cd catkin_workspace/src
$ git clone https://github.com/ajtudela/realsense_id_ros.git
$ cd ../
$ rosdep install --from-paths . --ignore-src
$ catkin_make
```

## Usage

To start the camera node in ROS:

	rosrun realsense_id_ros realsense_id_ros_node

## Nodes

### realsense_id_ros_node

Camera node to perform facial recognition.

#### Services

* **`authenticate`** ([realsense_id_ros/Authenticate])

	Perform one authentication. Returns an array of faces with users id. For example, you can trigger the computation from the console with

		rosservice call /realsense_id_ros_node/authenticate

* **`enroll`** ([realsense_id_ros/Enroll])

	Perform one enrollment for one new user. Returns an array of faces with users id. For example, you can trigger the computation from the console with

		rosservice call /realsense_id_ros_node/enroll

* **`remove_user`** ([realsense_id_ros/RemoveUser])

	Remove an user from the device. For example, you can trigger the computation from the console with

		rosservice call /realsense_id_ros_node/remove_user

* **`remove_all`** ([std_srvs/Empty])

	Remove all users from the device. For example, you can trigger the computation from the console with

		rosservice call /realsense_id_ros_node/remove_all

* **`query_users_id`** ([realsense_id_ros/QueryUsersId])

	Query the ids of the users. Returns the number of users and an array of ids. For example, you can trigger the computation from the console with

		rosservice call /realsense_id_ros_node/query_users_id

#### Parameters

* **`serial_port`** (string, default: "/dev/ttyACM0")

	Will attach to the device with the given serial port.


#### Reconfigure Parameters

* **`camera_rotation`** (int, default: "0")

	Enable the algorithm to work with a rotated device.

* **`security_level`** (string, default: "medium")

	Set security level to 'high' to allow no mask suport or to level 'medium' to support masks.

* **`algo_flow`** (string, default: "all")

	Algorithms which will be used during authentication: 'all', 'detection', 'recognition' and 'spoof'.

* **`face_selection_policy`** (string, default: "all")

	Face selection policy to run authentication on 'all' (up to 5) detected faces vs 'single' (closest) face.


## TODO list
- [ ] Use global dependencies.
- [ ] Preview snapshot of the faces.
- [x] Multiple faces.
- [ ] Host mode.
- [ ] Pair device.
- [ ] Secure mode.
- [ ] Option to use authentication loop with published topics.
- [x] Dynamic reconfigure server


[Intel RealSense ID]: https://www.intelrealsense.com/facial-authentication/
[Ubuntu]: https://ubuntu.com/
[ROS]: http://www.ros.org
[std_srvs/Empty]: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
[realsense_id_ros/Authenticate]: /srv/Authenticate.srv
[realsense_id_ros/Enroll]: /srv/Enroll.srv
[realsense_id_ros/RemoveUser]: /srv/RemoveUser.srv
[realsense_id_ros/QueryUsersId]: /srv/QueryUsersId.srv

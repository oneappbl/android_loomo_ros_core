# Android Loomo ROS Core

This repo allows you to connect a Loomo robot to an host PC running ROS. It exposes a number of the Loomo's sensors to ROS in ways that are compatible with the ROS Navigation stack.

This fork fixes a number of bugs I discovered while bringing up the navigation stack on Loomo. Most notably:

- Removes a buggy transform from the TF messages published from Loomo, which enables rotation correction of sensor data based on the platform angle
- Publish [nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html) messages as well as the /odom -> /base_link TF
- Changes data published from the ultrasonic sensor to a [sensor_msgs/Range](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html) message, which can be consumed by the Navigation stack range sensor plugin
- Publishes Realsense camera extrinsics as part of the TF tree
- Corrects timestamps published by the Loomo to better synchronize TF data with messages published from the Realsense camera
- Provides a basic "head scan" mechanism which pans the Loomo head back and forth to improve the FOV of the Realsense 
camera

## Building the Code

Please see [building.md](docs/building.md) for instructions on how to build the code.

## Running the code

Please see [usage.md](docs/usage.md) for instructions on how to use the code.

## Credit

This repo is based on the work done by Michael Everett and Jonathan P. How of the Aerospace Controls Laboratory at MIT, which was supported by the Ford Motor Company. For more information, see the original forked repository [here](https://github.com/mit-acl/android_loomo_ros_core).
# Building android_loomo_ros_core

Building the apk is very straightforward: Android Studio will download the dependency libraries for you automatically. 

## Prerequisites

1. Install [Android Studio](https://developer.android.com/studio)
1. Install the Android 5.1 SDK (API Level 22) and the Android NDK through the Android Studio SDK manager.

## Building

Open the project in Android Studio. It should automatically configure the project and begin downloading dependencies.

Click Build -> Make Project. Android Studio should build the project and produce an apk that can be installed on Loomo.

There's nothing more to it: ROS does not need to be installed to build this project as all the dependencies are pulled down via rosjava.
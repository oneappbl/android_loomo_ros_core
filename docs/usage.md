# Using android_loomo_ros_core

This document describes the process of setting up a Loomo and ROS PC so that both can communicate with each other.

You can use this node in either a USB tethered or Wifi tethered mode, depending on your use case. The USB tethered option requires some extra settings on the ROS master to ensure the Loomo can subscribe to topics.

## Prerequisites

1. A working installation of ROS Melodic. This code has only been tested against ROS Melodic running on Ubuntu 18.04, but it would conceivably work with ROS Kinectic, and ROS installations running on Windows 10. Follow the instructions on the [ROS Wiki](http://wiki.ros.org/ROS/Installation) for installing and setting up ROS.

1. A working NTP server running on the same machine as the ROS master. This is important: ROS is very particular about timestamps, and this apk will actively try to connect to an NTP server running on the ROS master. You will need to configure the NTP server to allow queries from the network that the Loomo is on.

1. Switch Loomo into Developer Mode (Settings > System > Loomo Developer > Developer Mode)
    - (I think this requires an internet connection, so join a wifi network now.)

3. You might need to switch Android into Developer Mode (tap some setting 7 times...look online for instructions).

## Install

1. Install the apk on Loomo, either by deploying it via Android Studio or using `adb install`

1. If that worked, there should be an app in Loomo's app drawer called `Loomo ROS`.

## Networking

### Wifi

1. The Loomo and the ROS master must be on the same Wifi network. It's best to use an IP address for the ROS master since name resolution on the Loomo has been a bit spotty.

### USB

1. Connect USB cable from Loomo's USB-C port to PC's USB-A port.

1. On Loomo, turn on USB Tethering (Settings > Wireless & Networks > More... > Tethering & portable hotspot > USB tethering)
On Ubuntu, you should see a popup from NetworkManager saying there's a new Wired Connection. You can rename it to Loomo if you want.

    At this point, the two devices are on a common network, and you should be able to ping one another.

1. You will almost certainly have to turn off the Loomo Wifi to make USB tethering work.

1. Enable USB tethering through the Android settings menu. You will have to re-enable tethering each time the USB cable is unplugged, or each time the Loomo boots.

1. Getting USB networking working will likely require you set the ROS_IP environment variable on the PC hosting the ROS master. Not having this variable set prevented the Loomo from subscribing to topics: it could publish, but wouldn't subscribe correctly.

## Launch

1. Start ROS master on the host PC (i.e. run `$ roscore`) (set the environment variable `ROS_MASTER_URI=http://<IP>:11311/`). Set <IP> to the IP of the host PC, on the correct network (USB or Wifi).

1. Tap the `Loomo ROS` app icon to launch the app. It should present you with a dialog that you can use to input the ROS master URL. Enter the ROS master URL (more on this later) and close the dialog. The app should now connect to the ROS master and begin publishing data.

1. You can enable and disable certain subsystems by chaning the toggle buttons on the homescreen. For safety reasons, the default configuration does not accept velocity commands on /cmd_vel. This must be enabled by toggling the "Sub vel" switch.
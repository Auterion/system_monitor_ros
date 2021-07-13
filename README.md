# system_monitor_ros
[![Build and Test package](https://github.com/Auterion/system_monitor_ros/workflows/Build%20and%20Test%20package/badge.svg?branch=master)](https://github.com/Auterion/system_monitor_ros/actions)

`system_monitor_ros` is a ROS 2 package to publish onboard companion status messages to the flight controller through [*px4_ros_com*](https://github.com/PX4/px4_ros_com) micro-RTPS bridge.
Similarly, this is a reference implementation of the Mavlink message [ONBOARD_COMPUTER_STATUS](https://mavlink.io/en/messages/common.html#ONBOARD_COMPUTER_STATUS), but following the message definition of the uORB message [`onboard_computer_status`](https://github.com/PX4/Firmware/blob/master/msg/onboard_computer_status.msg), which ROS counter-part is defined in [*px4_msgs*](https://github.com/PX4/px4_msgs/blob/master/msg/OnboardComputerStatus.msg). This same message is published by this node in the `/fmu/onboard_computer_status/in` topic.

## Requirements
  * ROS 2 Dashing or Eloquent - follow the [install guide](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/);
  * Colcon build tool: `apt install python3-colcon-common-extensions`
  * [*px4_ros_com*](https://github.com/PX4/px4_ros_com) built on the same workspace or on a different workspace that can be overlayed;
  * [*px4_msgs*](https://github.com/PX4/px4_msgs) built on the same workspace or on a different workspace that can be overlayed;

## Building
If *px4_ros_com* and *px4_msgs* were not built on another workspace, they can be built together with this package using the [`build_ros2_workspace.bash`](https://github.com/PX4/px4_ros_com/blob/master/scripts/build_ros2_workspace.bash) script for that purpose:

```
    $ mkdir -p colcon_ws/src
    $ cd colcon_ws/src
    $ git clone https://github.com/Auterion/system_monitor_ros.git -b ros2
    $ git clone https://github.com/PX4/px4_ros_com.git
    $ git clone https://github.com/PX4/px4_msgs.git
    $ ./px4_ros_com/scripts/build_ros2_workspace.bash
```

## Setup the workspace

```
    $ source colcon_ws/install/setup.bash
```

## Running the monitor node
The monitor can be run by the following `ros2 launch` command. The parameters can be edited in the `monitor_parameters.yaml` file.

```
    $ ros2 launch system_monitor_ros system_monitor.launch.py
```

To communicate with the autopilot, make sure that the `micrortps_agent` of *px4_ros_com* and the `micrortps_client` on the autopilot are both running.

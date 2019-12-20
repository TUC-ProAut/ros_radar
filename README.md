# ProAut Radar package

## Introduction

This package was designed to read messages from the <em>Bosch GPR V1.0 radar</em> using a <em>PEAK-USB</em> converter. This package is not about handling low-level can communication. Therefore it is assumed, that [https://wiki.ros.org/socketcan_bridge](socketcan_bridge_node) is already running.

The data from the sensor is published onto 'radar_messages' topic. Additionally, the measurements for each single target are republished as 'can_messages_A' and 'can_messages_B' respectively. For more details see [radar_pa_node](#radar_pa_node).

Moreover, it is possible to republish the data through a standard pointcloud on 'radar_pcd'. Please refer to [radar2pcd_pa_node](#radar2pcd_pa_node).


## radar_pa_node

```
rosrun radar_pa radar_pa_node
```

### Input and Output Topics:

Topic Name            | Type                                                                    | Description
----------------------|-------------------------------------------------------------------------|--------------------------------------------------------------------------
"received_messages"   | [can_msgs/Frame ](http://docs.ros.org/api/can_msgs/html/msg/Frame.html) | input from [https://wiki.ros.org/socketcan_bridge](socketcan_bridge_node)
"radar_messages"      | radar_pa_msgs/radar_msg                                                 | output


## radar2pcd_pa_node

```
rosrun radar_pa radar2pcd_pa_node
```

### Input and Output Topics:

Topic Name            | Type                                                                                            | Description
----------------------|-------------------------------------------------------------------------------------------------|------------------------------------------------
"radar_messages"      | radar_pa_msgs/radar_msg                                                                         | input from [radar_pa_node](#radar_pa_node)
"radar_pcd"           | [sensor_msgs/PointCloud ](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud.html) | output


## Links and packages

Source code at github:
> https://github.com/TUC-ProAut/ros_radar

Related packages:
> https://wiki.ros.org/socketcan_bridge

ROS packages: (upcoming)
> ros-kinetic-radar-pa <br>
> ros-melodic-radar-pa <br>


## ROS Build-Status and Documentation

ROS-Distribution | Build-Status                                                                                                                                                    | Documentation
-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------
Kinetic          | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__radar_pa__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdev__radar_pa__ubuntu_xenial_amd64/) | [docs.ros.org](http://docs.ros.org/kinetic/api/radar_pa/html/index.html)
Melodic          | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__radar_pa__ubuntu_xenial_amd64)](http://build.ros.org/job/Mdev__radar_pa__ubuntu_xenial_amd64/) | [docs.ros.org](http://docs.ros.org/melodic/api/radar_pa/html/index.html)

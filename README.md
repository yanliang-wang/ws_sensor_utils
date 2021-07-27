# Introduction

This workspace provides ROS drivers and calibration tools of sensor(IMU, LiDAR).

- [Introduction](#introduction)
- [Quick Start](#quick-start)
  - [Download & Build](#download--build)
  - [Drivers](#drivers)
    - [1. IMU](#1-imu)
    - [2. Velodyne](#2-velodyne)
    - [3. RSlidar](#3-rslidar)
  - [Calibration tools](#calibration-tools)
    - [1. Calibrate the noise and random walk noise of bias(IMU)](#1-calibrate-the-noise-and-random-walk-noise-of-biasimu)
    - [2. Calibrate the extrinsic parameters between LiDAR and IMU](#2-calibrate-the-extrinsic-parameters-between-lidar-and-imu)
- [Reference](#reference)

# Quick Start

## Download & Build

```bash
git clone --recursive https://github.com/yanliang-wang/driver_ws.git
```

Modify `imu_ws/src/code_utils/src/sumpixel_test.cpp`: `#include "backward.hpp"` --> `#include “code_utils/backward.hpp”`

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="code_utils"

catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

## Drivers

### 1. IMU

The default configuration can publish `sensors_msg/IMU` message including **orientation**, **angular_velocity**, **linear_acceleration**. So you can read the IMU message by run the following code.

```bash
roslaunch read_pkg view_imu.launch
```

If there is an error called `[ERROR] [1627211303.220970]: Fatal: could not find proper MT device.` May the following code will solve the problem.

```bash
sudo chmod 777 /dev/ttyUSB0
```

> Configure the IMU:
>
> 1. Xsens IMUs have much configuration we can modify, such as `baudrate`, `Synchronization settings`, `timeout`, etc. You can have an instruction by run the following code.
>
>    ```bash
>    rosrun xsens_driver mtdevice.py -h
>    ```
>
> 2. The most common configuration is to configure which kind of messages to publish. The following configuration is to set the driver to publish **orientation**, **linear_acceleration**, **angular_velocity**, which are very important information for robotic navigation.
>
>    ```bash
>    rosrun xsens_driver mtdevice.py --configure="oq,aa,wr"
>    ```
>

### 2. Velodyne

- Configure the network:

  Edit connections --> Edit --> IPv4 Settings( Method: Manual, Addresses: Add --> Address: 192.168.1.1, Netmask: 24) & modify Connection name --> save
- Get the velodyne xml file from https://github.com/Kitware/VeloView/tree/master/share and convert to a yaml file.
- Run it

```bash
roslaunch velodyne_pointcloud VLP16_points.launch
```

### 3. RSlidar

* Modify the config file in`driver_ws/src/driver_pkg/rslidar_sdk/config/` and modify the name of config file in the`start.launch`.

```bash
roslaunch rslidar_sdk start.launch
```

## Calibration tools

### 1. Calibrate the noise and random walk noise of bias(IMU)

Refer to [imu_utils](https://github.com/gaowenliang/imu_utils).

Step:

- collect the data while the IMU is Stationary, with a two hours duration, **at least two hours** suggested;
- modify the param of the launch file;

```xml
<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" type="string" value= "/imu/data"/>
        <param name="imu_name" type="string" value= "xsens"/>
        <param name="data_save_path" type="string" value= "$(find imu_utils)/data/"/>
        <param name="max_time_min" type="int" value= "120"/> <!-- the duration of your bag (Unit: minute) -->
        <param name="max_cluster" type="int" value= "100"/>
    </node>
</launch>
```

- roslaunch the rosnode and play the bag file;

```bash
roslaunch imu_utils xsens.launch

rosbag play -r 200 XXX.bag
```

- see the result;

The calibration result is saved in `imu_utils/data`.

### 2. Calibrate the extrinsic parameters between LiDAR and IMU

Refer to [lidar_imu_calib](https://github.com/chennuo0125-HIT/lidar_imu_calib.git).

Step:

- use rosbag tool record imu and lidar data(**Note: The pose for the sensors platform should be as diverse as possible, for example, a trajectory shaped like "8".**)

  ```bash
  rosbag record /imu /lidar_points
  ```

- config launch file

  ```xml
  lidar_topic: lidar data topic name
  imu_topic: imu data topic name
  bag_file: *.bag file record imu and lidar data topic
  ```

- start

  ```
  roslaunch lidar_imu_calib calib_exR_lidar2imu.launch
  ```

# Reference

1. [Ubuntu在ROS下安装Velodyne驱动](https://sunjiadai.xyz/blog/2019/01/07/Ubuntu%E5%9C%A8ROS%E4%B8%8B%E5%AE%89%E8%A3%85Velodyne%E9%A9%B1%E5%8A%A8/)
2. [Velodyne driver](https://github.com/ros-drivers/velodyne.git)
3. [Xsens IMU driver](https://github.com/ethz-asl/ethzasl_xsens_driver.git)
4. [rslidar driver](https://github.com/RoboSense-LiDAR/rslidar_sdk.git)
5. [lidar_imu_calib](https://github.com/chennuo0125-HIT/lidar_imu_calib.git)
6. [imu_utils](https://github.com/gaowenliang/imu_utils)




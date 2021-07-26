## Introduction

## Quick Start

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

## Reference

1. [Ubuntu在ROS下安装Velodyne驱动](https://sunjiadai.xyz/blog/2019/01/07/Ubuntu%E5%9C%A8ROS%E4%B8%8B%E5%AE%89%E8%A3%85Velodyne%E9%A9%B1%E5%8A%A8/)
2. [Velodyne driver](https://github.com/ros-drivers/velodyne.git)
3. [Xsens IMU driver](https://github.com/ethz-asl/ethzasl_xsens_driver.git)
4. [rslidar driver](https://github.com/RoboSense-LiDAR/rslidar_sdk.git)

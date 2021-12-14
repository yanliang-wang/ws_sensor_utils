# Hesai to Velodyne
A ros tool for converting Hesai pointcloud to Velodyne pointcloud format, which can be directly used for downstream algorithm, such as LOAM, LEGO-LOAM, LIO-SAM, etc.

## Useage

### 1. XYZIRT input
For **XYZIRT** format point clouds from `/hesai_points` (Notice that, you need the latest 
[HesaiLidar_General_ROS](https://github.com/HesaiTechnology/HesaiLidar_General_ROS) driver to get this type of point cloud):

```
rosrun hesai_to_velodyne hesai_to_velodyne XYZIRT XYZIRT
# or
rosrun hesai_to_velodyne hesai_to_velodyne XYZIRT XYZIR
# or
rosrun hesai_to_velodyne hesai_to_velodyne XYZIRT XYZI
```
The output point clouds are **XYZIRT** / **XYZIR** / **XYZI** point cloud `/velodyne_points` in Velodyne's format.

### 2. XYZI input
For **XYZI** format point clouds from `hesai_points`:
```
rosrun hesai_to_velodyne hesai_to_velodyne XYZI XYZIR
```
The output point clouds are **XYZIR** point cloud `/velodyne_points` in Velodyne's format.


## Subscribes
`/hesai_points`: sensor_msgs.PointCloud2, from hesai LiDAR.

## Publishes
`/velodyne_points`: sensor_msgs.PointCloud2, the frame_id is `velodyne`.

## Reference

1. [HesaiLidar_General_ROS](https://github.com/HesaiTechnology/HesaiLidar_General_ROS)


#pragma once

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

struct IOBoard {
  using Ptr = std::shared_ptr<IOBoard>;

  // input
  sensor_msgs::PointCloud2::ConstPtr input_cloud;

  // scan registration output
  sensor_msgs::PointCloud2::ConstPtr velodyne_cloud_2;          // _pubLaserCloud
  sensor_msgs::PointCloud2::ConstPtr laser_cloud_sharp;         // _pubCornerPointsSharp
  sensor_msgs::PointCloud2::ConstPtr laser_cloud_less_sharp;    // _pubCornerPointsLessSharp
  sensor_msgs::PointCloud2::ConstPtr laser_cloud_flat;          // _pubSurfPointsFlat
  sensor_msgs::PointCloud2::ConstPtr laser_cloud_less_flat;     // _pubSurfPointsLessFlat
  sensor_msgs::PointCloud2::ConstPtr imu_trans;                 // _pubImuTrans

  // laser odometry output
  sensor_msgs::PointCloud2::ConstPtr laser_cloud_corner_last;   // _pubLaserCloudCornerLast
  sensor_msgs::PointCloud2::ConstPtr laser_cloud_surf_last;     // _pubLaserCloudSurfLast
  sensor_msgs::PointCloud2::ConstPtr velodyne_cloud_3;          // _pubLaserCloudFullRes
  nav_msgs::Odometry::ConstPtr laser_odom_to_init;              // _pubLaserOdometry

  // laser mapping output
  sensor_msgs::PointCloud2::ConstPtr laser_cloud_surround;      // _pubLaserCloudSurround
  sensor_msgs::PointCloud2::ConstPtr velodyne_cloud_registered; // _pubLaserCloudFullRes
  nav_msgs::Odometry::ConstPtr aft_mapped_to_init;              // _pubOdomAftMapped

  // transform maintenance output
  nav_msgs::Odometry::ConstPtr integrated_to_init;              // _pubLaserOdometry2
};
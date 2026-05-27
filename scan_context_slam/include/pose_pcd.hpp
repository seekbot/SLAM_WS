#pragma once
#include "utilities.hpp"

struct PosePcd {
  PointCloudType      pcd_;                                             // stored in LiDAR frame
  Eigen::Matrix4d  pose_eig_           = Eigen::Matrix4d::Identity();   // raw odom
  Eigen::Matrix4d  pose_corrected_eig_ = Eigen::Matrix4d::Identity();   // post graph optimization
  double           timestamp_ = 0.0;
  int              idx_       = 0;
  bool             processed_ = false;                                  // loop-thread flag

  PosePcd() = default;
  PosePcd(const nav_msgs::msg::Odometry&     odom,
          const sensor_msgs::msg::PointCloud2& pc,
          int idx);
};

inline PosePcd::PosePcd(const nav_msgs::msg::Odometry&     odom,
                       const sensor_msgs::msg::PointCloud2& pc,
                       int idx)
{
  pose_eig_ = odomToEigen(odom);
  pose_corrected_eig_ = pose_eig_;                     // init: corrected == raw

  PointCloudType tmp;
  pcl::fromROSMsg(pc, tmp);

  /**
   * FAST-LIO publishes the cloud already in WORLD frame.
   * Store it in LiDAR/body frame so we can re-project later using
   * pose_corrected_eig_ once the graph corrects the pose.
   * 
   * Why: When a loop closure later corrects the pose, stored cloud is re-projected
   * with pose_corrected_eig_. Otherwise, it would be frozen at the wrong (pre-correction) 
   * position forever in the WORLD frame.
   */
  pcd_ = transformPcd(tmp, pose_eig_.inverse());

  timestamp_ = rclcpp::Time(odom.header.stamp).seconds();
  idx_       = idx;
}
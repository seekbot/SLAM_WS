#pragma once

// C++
#include <string>
#include <memory>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

// tf2 for Eigen and msg conversion
#include <tf2_eigen/tf2_eigen.hpp> 

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

// Eigen 
#include <Eigen/Dense>
#include <Eigen/Geometry>

// GTSAM 
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

/* === Eigen & GTSAM === */
// Eigen -> GTSAM
inline gtsam::Pose3 poseEigToGtsamPose(const Eigen::Matrix4d& T) {
  // Use the rotation matrix directly — no RPY loss near pitch = ±90°.
  gtsam::Rot3 R = gtsam::Rot3(Eigen::Quaterniond(T.block<3,3>(0,0)));
  gtsam::Point3 t(T(0,3), T(1,3), T(2,3));
  return gtsam::Pose3(R, t);
}

// GTSAM -> Eigen
inline Eigen::Matrix4d gtsamPoseToPoseEig(const gtsam::Pose3& g) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = g.rotation().matrix();       // direct 3x3, no RPY
  T(0,3) = g.translation().x();
  T(1,3) = g.translation().y();
  T(2,3) = g.translation().z();
  return T;
}

/* === ROS2 Odometry -> Eigen === */
inline Eigen::Matrix4d odomToEigen(const nav_msgs::msg::Odometry& odom) {
  Eigen::Quaterniond q(odom.pose.pose.orientation.w,    // w first in Eigen!
                       odom.pose.pose.orientation.x,
                       odom.pose.pose.orientation.y,
                       odom.pose.pose.orientation.z);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = q.normalized().toRotationMatrix();
  T(0,3) = odom.pose.pose.position.x;
  T(1,3) = odom.pose.pose.position.y;
  T(2,3) = odom.pose.pose.position.z;
  return T;
}

/* === PCL (Cloud helpers) === */
// Transform pcd with Eigen transformation matrix
inline PointCloudType transformPcd(const PointCloudType& in,
                                const Eigen::Matrix4d& T) {
  if (in.empty()) return in;
  PointCloudType out;
  pcl::transformPointCloud(in, out, T);
  return out;
}

// Discretize point cloud as voxel
inline PointCloudType::Ptr voxelizePcd(const PointCloudType& in, float leaf) {
  PointCloudType::Ptr in_ptr(new PointCloudType(in));
  PointCloudType::Ptr out  (new PointCloudType);
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf, leaf, leaf);
  vg.setInputCloud(in_ptr);
  vg.filter(*out);
  return out;
}
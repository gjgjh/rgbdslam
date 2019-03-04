/*******************************************************
 * Copyright (C) 2019, Navigation And Location Group, Peking University
 * 
 * This file is part of rgbdslam.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: GJH (guojh_rs@pku.edu.cn)
 *******************************************************/

#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// define the commonly included file to avoid a long include list
// for std
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <boost/timer.hpp>
#include <string>
#include <sstream>
#include <queue>
#include <memory>
#include <map>
#include <algorithm>

using namespace std;

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Isometry3d;
using Eigen::AngleAxisd;

// for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// for pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using pcl::transformPointCloud;
using pcl::io::savePCDFileBinary;
using pcl::visualization::CloudViewer;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// for g2o
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>

// for ros
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>

#endif // COMMON_INCLUDE_H
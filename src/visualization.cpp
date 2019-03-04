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

#include "rgbdslam/visualization.h"
#include "rgbdslam/config.h"

namespace rgbdslam
{
nav_msgs::Path path; // define global variable in the implementation file
ros::Publisher path_pub;
ros::Publisher campose_pub;
ros::Publisher color_pub;
ros::Publisher depth_pub;
ros::Publisher map_pub;
string outputpath_;

void registerNodeHandle(ros::NodeHandle &nh,string outputpath)
{
path_pub=nh.advertise<nav_msgs::Path>("/rgbdslam/Path", 1000, true); // latch=true
campose_pub=nh.advertise<geometry_msgs::PoseStamped>("/rgbdslam/CamPose", 1000);
color_pub=nh.advertise<sensor_msgs::Image>("/rgbdslam/color", 1000);
depth_pub=nh.advertise<sensor_msgs::Image>("/rgbdslam/depth", 1000);
map_pub=nh.advertise<PointCloud>("/rgbdslam/Map", 1000, true);

outputpath_=outputpath;
}

void pubPath(const Isometry3d& pose,const double& timestamp)
{
    geometry_msgs::PoseStamped poseSt;
    string frame_id=Config::getConfig()->get<string>("frameID");
    poseSt.header.frame_id = frame_id;
    ros::Time tmp;
    poseSt.header.stamp=tmp.fromSec(timestamp);
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    Isometry3d posetemp =  (pose.inverse());

    poseSt.pose.position.x = posetemp.translation()(0, 0);
    poseSt.pose.position.y = posetemp.translation()(1, 0);
    poseSt.pose.position.z = posetemp.translation()(2, 0);

    Eigen::Quaterniond q = Eigen::Quaterniond(posetemp.rotation());

    poseSt.pose.orientation.z = q.z();
    poseSt.pose.orientation.y = q.y();
    poseSt.pose.orientation.x = q.x();
    poseSt.pose.orientation.w = q.w();
    path.poses.push_back(poseSt);

    campose_pub.publish(poseSt);
    path_pub.publish(path);
}


void pubColorImg(const cv::Mat& color)
{
    cv_bridge::CvImage img_bridge;
    img_bridge.image = color;
    img_bridge.encoding = "bgr8";
    img_bridge.header.stamp = ros::Time::now();
    sensor_msgs::Image ros_image;
    img_bridge.toImageMsg(ros_image);

    color_pub.publish(ros_image);
}

void pubDepthImg(const cv::Mat& depth)
{
    cv_bridge::CvImage img_bridge;
    img_bridge.image = depth;
    img_bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    img_bridge.header.stamp = ros::Time::now();
    sensor_msgs::Image ros_image;
    img_bridge.toImageMsg(ros_image);

    depth_pub.publish(ros_image);
}

void pubMap(const PointCloud::Ptr& pCloud)
{
    string frame_id = Config::getConfig()->get<string>("frameID");
    pCloud->header.frame_id = frame_id;
    pcl_conversions::toPCL(ros::Time::now(), pCloud->header.stamp);

    map_pub.publish(pCloud);
}

}
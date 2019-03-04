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

#ifndef VISUALIZATION_H
#define VISUALIZATION_H
#include "rgbdslam/common_include.h"

namespace rgbdslam
{
extern ros::Publisher path_pub; // declare global variable as extern in the header file
extern ros::Publisher campose_pub;
extern ros::Publisher color_pub;
extern ros::Publisher depth_pub;
extern ros::Publisher map_pub;
extern string outputpath_;

void registerNodeHandle(ros::NodeHandle &nh,string outputpath);
void pubPath(const Isometry3d& pose,const double& timestamp);
void pubColorImg(const cv::Mat& color);
void pubDepthImg(const cv::Mat& depth);
void pubMap(const PointCloud::Ptr& pCloud);
}
#endif // VISUALIZATION_H
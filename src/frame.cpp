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

#include "rgbdslam/frame.h"
#include "rgbdslam/config.h"

namespace rgbdslam
{
pcl::VoxelGrid<PointT> voxel;
pcl::PassThrough<PointT> pass;
PointCloud::Ptr tmp(new PointCloud);

Frame::Frame()
:id_(-1),rgb_time_stamp_(-1),d_time_stamp_(-1),camera_(nullptr),localMap_(nullptr)
{

}

Frame::Frame(long id, double rgb_time_stamp,double d_time_stamp,
             Isometry3d T_c_w,Camera::Ptr camera,
             PointCloud::Ptr localMap,cv::Mat color,cv::Mat depth)
      :id_(id),rgb_time_stamp_(rgb_time_stamp),d_time_stamp_(d_time_stamp),
      T_c_w_(T_c_w),camera_(camera),localMap_(localMap),color_(color),depth_(depth){

    PointCloud::Ptr pCloud(new PointCloud);
    localMap_=pCloud;
    desp_=cv::Mat();
}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return Frame::Ptr( new Frame(factory_id++) );
}

void Frame::image2PointCloud()const
{
    for (int m = 0; m < depth_.rows; m++) {
        for (int n = 0; n < depth_.cols; n++) {
            float d = depth_.ptr<float>(m)[n];
            if (!(d > 0))
                continue;
            PointT p;

            p.z = double(d) / camera_->scale(); // factor = 1
            p.x = (n - camera_->cx()) * p.z / camera_->fx();
            p.y = (m - camera_->cy()) * p.z / camera_->fy();

            p.b = color_.ptr<uchar>(m)[n * 3];
            p.g = color_.ptr<uchar>(m)[n * 3 + 1];
            p.r = color_.ptr<uchar>(m)[n * 3 + 2];

            localMap_->points.push_back(p);
        }
    }

    // voxel filter
    double gridsize=Config::instance()->get<double>("voxel_grid");
    double max_dist_pointcloud=Config::instance()->get<double>("max_dist_pointcloud");
    voxel.setLeafSize(gridsize, gridsize, gridsize);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, max_dist_pointcloud);

    voxel.setInputCloud(localMap_);
    voxel.filter(*tmp);
    pass.setInputCloud(tmp);
    pass.filter(*localMap_);

    localMap_->is_dense = false;
}

void Frame::computeKptAndDesp(){
    cv::Ptr<cv::ORB> orb=cv::ORB::create();
    orb->detectAndCompute(color_,cv::Mat(),kp_,desp_);
}

}
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

#ifndef FRAME_H
#define FRAME_H

#include "rgbdslam/common_include.h"
#include "rgbdslam/camera.h"

#include <DBoW3/DBoW3.h>

namespace rgbdslam
{
class Frame{
public:
    typedef std::shared_ptr<Frame> Ptr;

    Frame();
    Frame(long id, double rgb_time_stamp=0,double d_time_stamp=0,
            Isometry3d T_c_w=Isometry3d::Identity(),Camera::Ptr camera= nullptr,
            PointCloud::Ptr localMap= nullptr,cv::Mat color=cv::Mat(),cv::Mat depth=cv::Mat());
    ~Frame();

    // factory function
    static Frame::Ptr createFrame();

    // interface
    unsigned long id()const{return id_;}
    PointCloud::Ptr localMap()const{return localMap_;}
    double& rgb_time_stamp(){return rgb_time_stamp_;}
    double& d_time_stamp(){return d_time_stamp_;}
    cv::Mat& color(){return color_;}
    cv::Mat& depth(){return depth_;}
    Camera::Ptr& camera(){return camera_;}
    Isometry3d& T_c_w(){return T_c_w_;}
    const cv::Mat& desp()const{return desp_;}
    const vector<cv::KeyPoint>& kp()const{return kp_;}
    DBoW3::BowVector& bow_vec(){ return bow_vec_;}

    void image2PointCloud()const;
    void computeKptAndDesp();

private:
    unsigned long id_;          // id of this frame
    double rgb_time_stamp_;     // when color image is recorded
    double d_time_stamp_;       // when depth image is recorded
    Isometry3d T_c_w_;          // transform from world to camera
    Camera::Ptr camera_;        // Pinhole RGBD camera model
    PointCloud::Ptr localMap_;  // local map point cloud in world
    cv::Mat color_,depth_;      // color and depth image
    cv::Mat desp_;              // descriptors
    vector<cv::KeyPoint> kp_;   // orb keypoints
    DBoW3::BowVector bow_vec_;  // BoW vector
};
}

#endif // FRAME_H
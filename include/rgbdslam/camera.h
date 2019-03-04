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

#ifndef CAMERA_H
#define CAMERA_H

#include "rgbdslam/common_include.h"

namespace rgbdslam
{
// Pinhole RGBD camera model
class Camera{
public:
    typedef std::shared_ptr<Camera> Ptr;

    Camera(){}
    ~Camera(){}
    Camera(double fx,double cx,double fy,double cy,double scale=0):
    fx_(fx),cx_(cx),fy_(fy),cy_(cy),scale_(scale){}

    cv::Point2f pixel2cam(const cv::Point2f &p)const
    {
        return cv::Point2f((p.x - cx_) / fx_,(p.y - cy_) / fy_);
    }

    // interface
    double cx()const{ return cx_;}
    double cy()const{ return cy_;}
    double fx()const{ return fx_;}
    double fy()const{ return fy_;}
    double scale()const{ return scale_;}
private:
    double cx_,cy_,fx_,fy_,scale_; // Camera intrinsics
};

}

#endif // CAMERA_H
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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "rgbdslam/visual_odometry.h"
#include "rgbdslam/visualization.h"
#include "rgbdslam/common_include.h"
#include "rgbdslam/camera.h"
#include "rgbdslam/config.h"

using namespace sensor_msgs;
using namespace message_filters;

rgbdslam::Camera::Ptr camera(new rgbdslam::Camera);
ros::Subscriber caminfo_sub;
rgbdslam::VisualOdometry::Ptr vo;
int numFrames=0;

void callback(const ImageConstPtr &msg1, const ImageConstPtr &msg2);
void caminfocb(const sensor_msgs::CameraInfo::ConstPtr &msg);
int main(int argc, char** argv)
{
    rgbdslam::config_path=argv[1];
    rgbdslam::output_path=argv[2];
    rgbdslam::vocab_path=argv[3];

    // load config file
    rgbdslam::Config::instance()->setParameterFile(rgbdslam::config_path);

    rgbdslam::VisualOdometry::Ptr vo_tmp(new rgbdslam::VisualOdometry);
    vo=vo_tmp;

    ros::init(argc, argv, "rgbdslam_node");
    ROS_INFO("================================================");
    ROS_INFO("RGBDSLAM-DEMO ");
    ROS_INFO("Copyright (C) 2019");
    ROS_INFO("Navigation And Location Group, Peking University");
    ROS_INFO("Starting...");
    ROS_INFO("================================================");
    ros::NodeHandle nh;
    rgbdslam::registerNodeHandle(nh);

    // time synchronize. see: http://wiki.ros.org/message_filters#Time_Synchronizer
    Subscriber<Image> color_sub(nh, "/camera/rgb/image_color", 1000,ros::TransportHints().tcpNoDelay());
    Subscriber<Image> depth_sub(nh, "/camera/depth/image", 1000,ros::TransportHints().tcpNoDelay());
    caminfo_sub = nh.subscribe("/camera/rgb/camera_info", 1000,caminfocb);
    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), color_sub, depth_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();

    return 0;
}

void callback(const ImageConstPtr &msg1, const ImageConstPtr &msg2)
{
    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg1);
    cv::Mat rgb = cvPtr->image;

    std::stringstream ss;
    ss << cvPtr->header.stamp.sec << "." << cvPtr->header.stamp.nsec;
    double rgbtimestamp;
    ss >> rgbtimestamp;

    cv_bridge::CvImagePtr cvPtr2 =
            cv_bridge::toCvCopy(msg2,sensor_msgs::image_encodings::TYPE_32FC1); // important! 32 bit float (meters)
    cv::Mat depth = cvPtr2->image;

    std::stringstream ss2;
    ss2 << cvPtr2->header.stamp.sec << "." << cvPtr2->header.stamp.nsec;
    double depthtimestamp;
    ss2 >> depthtimestamp;

    /*
    std::cerr<<std::to_string(rgbtimestamp)<<" - "<<std::to_string(depthtimestamp)<<std::endl;
    cv::imshow("rgb",rgb);
    cv::waitKey(1);
    cv::imshow("depth",depth);
    cv::waitKey(1);
    */

    rgbdslam::Frame::Ptr pFrame=rgbdslam::Frame::createFrame();
    pFrame->camera()=camera;
    pFrame->rgb_time_stamp()=rgbtimestamp;
    pFrame->d_time_stamp()=depthtimestamp;
    pFrame->color()=rgb;
    pFrame->depth()=depth;
    pFrame->image2PointCloud();

    vo->addFrame(pFrame);

    static int optInterval=rgbdslam::Config::instance()->get<int>("optimization_interval");
    numFrames++;
    if(numFrames%optInterval==0)
        vo->optimizeMap();

}

void caminfocb(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    // https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
    rgbdslam::Camera::Ptr temp(new rgbdslam::Camera(
            msg->K.data()[0],
            msg->K.data()[2],
            msg->K.data()[4],
            msg->K.data()[5],
            1 ));
    camera=temp;

    if(camera!= nullptr) caminfo_sub.shutdown(); // only subscribe once
}

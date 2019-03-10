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

#include "rgbdslam/visualization.h"
#include "rgbdslam/common_include.h"
#include "rgbdslam/config.h"

ros::Subscriber map_sub;
ros::Subscriber path_sub;
void mapcb(const PointCloud::Ptr& msg);
void pathcb(const nav_msgs::PathConstPtr& msg);
int main(int argc,char** argv)
{
    rgbdslam::output_path=string(argv[1]);

    ros::init(argc, argv, "map_saver_node");
    ros::NodeHandle nh2;
    rgbdslam::registerNodeHandle(nh2);
    map_sub=nh2.subscribe("/rgbdslam/Map", 1000,mapcb);
    path_sub=nh2.subscribe("/rgbdslam/Path",1000,pathcb);

    ros::spin();

    return 0;
}

void mapcb(const PointCloud::Ptr& msg){
    PointCloud::Ptr globalMap=msg;
    pcl::io::savePCDFileASCII (rgbdslam::output_path+"/globalMap.pcd", *globalMap);

    ROS_INFO("============================================");
    ROS_INFO("Saved global map to result/globalMap.pcd");
    ROS_INFO("Done!");
    ROS_INFO("============================================");
    map_sub.shutdown();
}

void pathcb(const nav_msgs::PathConstPtr& msg){
    ofstream fout((rgbdslam::output_path+"/CameraPoses.txt").c_str());

    for(auto& p:msg->poses){
        fout <<to_string(p.header.stamp.toSec())
        <<" "<<p.pose.position.x
        <<" "<<p.pose.position.y
        <<" "<<p.pose.position.z
        <<" "<<p.pose.orientation.x
        <<" "<<p.pose.orientation.y
        <<" "<<p.pose.orientation.z
        <<" "<<p.pose.orientation.w
        <<endl;
    }

    ROS_INFO("============================================");
    ROS_INFO("Saved trajectories to result/CameraPoses.txt");
    ROS_INFO("Done!");
    ROS_INFO("============================================");
    fout.close();
    path_sub.shutdown();
}
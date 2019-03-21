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

#include "rgbdslam/map.h"

namespace rgbdslam
{
Map::Map()
{
    PointCloud::Ptr pCloud(new PointCloud);
    globalMap_= pCloud;
}

Map::~Map()
{
    globalMap_->clear();
}

void Map::insertKeyFrame(Frame::Ptr frame)
{
    keyframes_.insert({frame->id(),frame});
    expandmap(frame);
}

void Map::updateMap()const
{
    globalMap_->clear();

    for(auto& kf:keyframes_)
        expandmap(kf.second);
}

void Map::expandmap(Frame::Ptr frame)const
{
    PointCloud::Ptr temp(new PointCloud);
    pcl::transformPointCloud(*frame->localMap(), *temp, frame->T_c_w().inverse().matrix());
    *globalMap_+=*temp;
}

}
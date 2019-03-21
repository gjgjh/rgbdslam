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

#ifndef MAP_H
#define MAP_H
#include "rgbdslam/common_include.h"
#include "rgbdslam/frame.h"

namespace rgbdslam
{
class Map{
public:
    typedef std::shared_ptr<Map> Ptr;

    Map();
    ~Map();

    void insertKeyFrame(Frame::Ptr frame);
    void updateMap()const;
    void expandmap(Frame::Ptr frame)const;
    const PointCloud::Ptr& globalMap()const { return globalMap_;}
    const unordered_map<unsigned long,Frame::Ptr>& keyframes()const { return keyframes_;}
private:
    PointCloud::Ptr globalMap_;                                // all map point cloud
    unordered_map<unsigned long,Frame::Ptr> keyframes_;        // all key frames

};
}

#endif // MAP_H

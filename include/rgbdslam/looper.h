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

#ifndef LOOPER_H
#define LOOPER_H

#include "rgbdslam/common_include.h"
#include "rgbdslam/config.h"
#include "rgbdslam/frame.h"

#include <DBoW3/DBoW3.h>

namespace rgbdslam
{
class Looper{
public:
    typedef std::shared_ptr<Looper> Ptr;

    Looper();

    // interface
    void calBowVec(Frame::Ptr pFrame)const;
    vector<Frame::Ptr> getPossibleLoops(const Frame::Ptr& pFrame,
            const Map::Ptr& pMap)const;
private:
    DBoW3::Vocabulary vocab_;					// vocabulary file
    double min_sim_score_;						// minimum score when considered as the same scene
    int min_interval_;							// loop closure's minimum keyframe interval
};
}

#endif // LOOPER_H
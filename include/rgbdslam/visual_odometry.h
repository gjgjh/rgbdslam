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

#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "rgbdslam/common_include.h"
#include "rgbdslam/map.h"
#include "rgbdslam/looper.h"

namespace rgbdslam
{
class VisualOdometry{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState{INITIALIZING=-1,OK=0,LOST};

    VisualOdometry();
    ~VisualOdometry();

    bool addFrame(Frame::Ptr frame);        // add a new frame
    void optimizeMap();                     // pose graph optimization for map_

private:
    VOState state_;                         // current VO status
    Map::Ptr map_;                          // map with all keyframes and global point cloud map
    Frame::Ptr ref_;                        // reference key frame
    Frame::Ptr curr_;                       // current frame

    vector<cv::DMatch> feature_matches_;    // feature matches

    Isometry3d T_c_r_estimated_;            // the estimated pose of current frame
    int num_inliers_;                       // number of inlier features in PnP
    int num_lost_;                          // number of lost times

    // parameters
    int max_num_lost_;                      // max number of continuous lost times
    int min_inliers_;                       // minimum inliers

    double max_norm_;                       // maximum norm of two key-frames
    double keyframe_threshold_;             // minimum norm of two key-frames

    bool check_loop_closure_;               // whether check loop closure or not
    bool isLoops_;                          // whether ref_ and curr_ are loop closure
    Looper::Ptr looper_;                    // loop closure

    // inner operation
    void featureMatching();
    void poseEstimationPnP();

    double normofT(Isometry3d& T);
    bool checkEstimatedPose();
    bool checkKeyFrame();
    void addKeyFrame();
    
    void checkLoops();
};

}

#endif // VISUAL_ODOMETRY_H
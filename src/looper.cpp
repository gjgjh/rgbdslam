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
#include "rgbdslam/looper.h"

namespace rgbdslam
{
Looper::Looper(){
    vocab_.load(rgbdslam::vocab_path);
    if(vocab_.empty())
        cerr<<"Vocabulary does not exist\n";

    min_sim_score_=Config::instance()->get<double>("min_sim_score");
    min_interval_=Config::instance()->get<int>("min_interval");
}

void Looper::calBowVec(Frame::Ptr pFrame)const{
    vocab_.transform(pFrame->desp(),pFrame->bow_vec());
}

vector<Frame::Ptr> Looper::getPossibleLoops(const Frame::Ptr& pFrame,
        const Map::Ptr& pMap)const
{
    vector<Frame::Ptr> res;
    double score;
    for(auto& kf:pMap->keyframes()){
        if(abs(pFrame->id()-kf.first)>min_interval_){
            score=vocab_.score(kf.second->bow_vec(),pFrame->bow_vec());
            if(score>min_sim_score_)
                res.push_back(kf.second);
        }
    }
    return res;
}

}
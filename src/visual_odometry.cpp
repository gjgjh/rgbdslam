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

#include "rgbdslam/visual_odometry.h"
#include "rgbdslam/config.h"
#include "rgbdslam/visualization.h"
#include "rgbdslam/globalOpt.h"

namespace rgbdslam
{
GlobalOptimization globalEstimator;

VisualOdometry::VisualOdometry():
state_(INITIALIZING),ref_(nullptr),curr_(nullptr),map_(new Map),
num_lost_(0),num_inliers_(0),looper_(new Looper)
{
    max_num_lost_=Config::getConfig()->get<int>("max_num_lost");
    min_inliers_=Config::getConfig()->get<int>("min_inliers");
    max_norm_=Config::getConfig()->get<double>("max_norm");
    keyframe_threshold_=Config::getConfig()->get<double>("keyframe_threshold");
    check_loop_closure_=Config::getConfig()->get<string>("check_loop_closure")=="true";
    isLoops_=false;
}

VisualOdometry::~VisualOdometry()
{

}

bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    switch (state_)
    {
        case INITIALIZING:
        {
            state_=OK;
            curr_=ref_=frame;
            map_->insertKeyFrame(frame);
            // extract features from first frame
            curr_->computeKptAndDesp();
            looper_->calBowVec(curr_);

            auto* v=new g2o::VertexSE3;
            v->setId(curr_->id());
            v->setEstimate(curr_->T_c_w());
            v->setFixed(true);
            globalEstimator.optimizer_.addVertex(v);

            rgbdslam::pubPath(curr_->T_c_w(),curr_->d_time_stamp());
            rgbdslam::pubColorImg(curr_->color());
            rgbdslam::pubDepthImg(curr_->depth());

            break;
        }
        case OK:
        {
            curr_=frame;
            curr_->computeKptAndDesp();
            looper_->calBowVec(curr_);
            featureMatching();
            poseEstimationPnP();
            if(checkEstimatedPose()==true) // a good estimation
            {
                curr_->T_c_w()=T_c_r_estimated_*ref_->T_c_w(); // T_c_w=T_c_r*T_r_w
                num_lost_=0;
                cout << "current frame ID " << curr_->id() << " : " << endl <<
                    curr_->T_c_w().matrix()<<endl;

                rgbdslam::pubPath(curr_->T_c_w(),curr_->d_time_stamp()); // TUM uses depth timestamp when evaluating
                rgbdslam::pubColorImg(curr_->color());
                rgbdslam::pubDepthImg(curr_->depth());

                if(checkKeyFrame()== true) // is a keyframe
                {
                    if(check_loop_closure_)
                        checkLoops();
                    ref_=curr_;
                    addKeyFrame();
                    rgbdslam::pubMap(map_->globalMap());
                }
            } else{ // bad estimation
                num_lost_++;
                if(num_lost_>max_num_lost_)
                {
                    state_=LOST;
                }
                return false;
            }
            break;
        }
        case LOST:
        {
            cout<<"vo has lost\n";
            break;
        }
    }
    return true;
}

void VisualOdometry::featureMatching()
{
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(ref_->desp(), curr_->desp(), matches);

    double min_dist = 9999, max_dist = 0;
    for (int i = 0; i < ref_->desp().rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    feature_matches_.clear(); // clear the last matching result
    for (int i = 0; i < ref_->desp().rows; i++) {
        if (matches[i].distance <= max(2 * min_dist, 30.0)) {
            feature_matches_.push_back(matches[i]);
        }
    }
}

void VisualOdometry::poseEstimationPnP()
{
    if(num_inliers_==-1)
    {
        num_inliers_=0;
        return;
    }

    cv::Mat K = (cv::Mat_<double>(3, 3)
            << ref_->camera()->fx(), 0, ref_->camera()->cx(),
            0, ref_->camera()->fy(), ref_->camera()->cy(),
            0, 0, 1);
    vector<cv::Point3f> pts_3d;
    vector<cv::Point2f> pts_2d;
    for (cv::DMatch m:feature_matches_) {
        float d = ref_->depth().ptr<float>(int(ref_->kp()[m.queryIdx].pt.y))[
                int(ref_->kp()[m.queryIdx].pt.x)];
        if (!(d > 0))
            continue;
        float dd = d / ref_->camera()->scale();
        cv::Point2d p1 =ref_->camera()->pixel2cam(ref_->kp()[m.queryIdx].pt);
        pts_3d.push_back(cv::Point3f(p1.x * dd, p1.y * dd, dd));
        pts_2d.push_back(curr_->kp()[m.trainIdx].pt);
    }

    if (pts_3d.size() < 5 || pts_2d.size() < 5) {
        num_inliers_ = -1;
        return;
    }

    cv::Mat r, t, inliers;
    cv::solvePnPRansac(pts_3d, pts_2d, K, cv::Mat(), r, t, false, 100, 8.0, 0.999, inliers, cv::SOLVEPNP_EPNP);
    num_inliers_=inliers.rows;

    cv::Mat R_tmp;
    cv::Rodrigues(r, R_tmp);
    Matrix3d R;
    cv2eigen(R_tmp, R);
    Vector3d tvec;
    cv2eigen(t, tvec);

    Isometry3d T = Isometry3d::Identity();
    AngleAxisd rvec(R);
    T.rotate(rvec);
    T.pretranslate(tvec);

    T_c_r_estimated_=T;

    /*
    // get RANSAC matches
    vector<cv::DMatch> inlier_matches;
    for (int i = 0; i < inliers.rows; i++) {
        inlier_matches.push_back(feature_matches_[inliers.ptr<int>(i)[0]]);
    }
    cv::Mat img_goodmatch,img_inliermatch;
    drawMatches(ref_->color(),ref_->kp(),curr_->color(),curr_->kp(),feature_matches_,img_goodmatch);
    drawMatches(ref_->color(),ref_->kp(),curr_->color(),curr_->kp(),inlier_matches,img_inliermatch);
    cv::imwrite(to_string(ref_->id())+".png",ref_->color());
    cv::imwrite(to_string(curr_->id())+".png",curr_->color());
    cv::imwrite(to_string(ref_->id()) + "_" + to_string(curr_->id()) + "goodmatch.png", img_goodmatch);
    cv::imwrite(to_string(ref_->id()) + "_" + to_string(curr_->id()) + "inliermatch.png", img_inliermatch);
    */
}

void VisualOdometry::addKeyFrame()
{
    cout<<"adding keyframe "<<curr_->id()<<endl<<endl;
    map_->insertKeyFrame ( curr_ );
}

bool VisualOdometry::checkEstimatedPose()
{
    if (feature_matches_.size() <= 5) {
        num_inliers_ = -1;
    }

    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_)
    {
        cout<<"reject because inlier is too small: "<<ref_->id()<<"-"
            <<curr_->id()<<endl<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    if ( normofT(T_c_r_estimated_) > max_norm_)
    {
        cout<<"reject because motion is too large: "<<ref_->id()<<"-"
            <<curr_->id()<<endl<<endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    if ( normofT(T_c_r_estimated_) <keyframe_threshold_ )
    {
        cout<<"reject because motion is too close \n\n";
        return false;
    }

    // add vertices (current new keyframe only, because old keyframe already has a vetex)
    if(!isLoops_)
    {
        auto* v=new g2o::VertexSE3;
        v->setId(curr_->id());
        v->setEstimate(curr_->T_c_w());
        globalEstimator.optimizer_.addVertex(v);
    }

    // add edges
    g2o::EdgeSE3* e=new g2o::EdgeSE3;
    e->setVertex(0,globalEstimator.optimizer_.vertices()[ref_->id()]);
    e->setVertex(1,globalEstimator.optimizer_.vertices()[curr_->id()]);
    e->setRobustKernel(globalEstimator.robustKernel_);
    e->setMeasurement(T_c_r_estimated_.inverse());
    e->setInformation(Eigen::Matrix<double,6,6>::Identity()*1/0.01);
    globalEstimator.optimizer_.addEdge(e);

    return true;
}

double VisualOdometry::normofT(Isometry3d& T)
{
    AngleAxisd rvec(T.rotation());
    Vector3d tvec(T.translation());
    double tNorm = tvec.norm();
    double rNorm = rvec.angle();

    return fabs(min(rNorm, 2 * M_PI - rNorm)) + fabs(tNorm);
}

void VisualOdometry::checkLoops()
{
    isLoops_=true;
    auto loops=looper_->getPossibleLoops(curr_,map_);
    for(auto& kf:loops){
        ref_=kf;
        featureMatching();
        poseEstimationPnP();
        if(checkEstimatedPose()==true)
            checkKeyFrame();

        cout<<"Loops: "<<ref_->id()<<" - "<<curr_->id()<<endl;
    }

    isLoops_= false;
}

void VisualOdometry::optimizeMap()
{
    // pose graph optimization
    globalEstimator.saveOptResult(rgbdslam::output_path+"/G2oBefore.g2o");
    globalEstimator.optimize();
    globalEstimator.saveOptResult(rgbdslam::output_path+"/G2oAfter.g2o");

    // update key frame poses
    ofstream fout((rgbdslam::output_path+"/KeyframePosesAfterG2o.txt").c_str());
    for(auto& kf:map_->keyframes())
    {
        auto* vertex= dynamic_cast<g2o::VertexSE3*>
                (globalEstimator.optimizer_.vertices()[kf->id()]);
        Isometry3d pose=vertex->estimate();
        fout<<"Frame ID "<<kf->id()<<" : "<<endl<<pose.inverse().matrix()<<endl<<endl;
        kf->T_c_w()=pose.inverse();
    }
    fout.close();

    // update point cloud map
    map_->updateMap();
}

}

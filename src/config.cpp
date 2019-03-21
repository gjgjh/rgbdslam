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

#include "rgbdslam/config.h"

namespace rgbdslam
{
    std::shared_ptr<Config> Config::instance(){
        static std::shared_ptr<Config> config_= nullptr;
        if (config_ == nullptr)
            config_ = std::shared_ptr<Config>(new Config);

        return config_;
    }

    void Config::setParameterFile(const std::string &filename) {
        file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        if (!file_.isOpened()) {
            cerr << "parameter file " << filename << " doesn't exist.\n";
            file_.release();
        }
    }


//    template<typename T>
//    T Config::get(const std::string &key)const{
//        return T(file_[key]);
//    }

    Config::~Config() {
        if (file_.isOpened())
            file_.release();
    }

}
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

#ifndef CONFIG_H_
#define CONFIG_H_
#include "rgbdslam/common_include.h"

namespace rgbdslam {
// a Singleton class
class Config {
public:
    ~Config();
    Config(const Config&)= delete;
    Config& operator=(const Config&)= delete;

    static std::shared_ptr<Config> instance();
    void setParameterFile(const std::string &filename);
    template<typename T>
    T get(const std::string &key)const{
        return T(file_[key]);
    }
private:
    Config(){}

    cv::FileStorage file_;
};
}

#endif // CONFIG_H
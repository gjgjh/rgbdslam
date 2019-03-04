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

#ifndef GLOBALOPT_H
#define GLOBALOPT_H
#include "rgbdslam/common_include.h"

namespace rgbdslam
{
class GlobalOptimization{
public:
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> SlamBlockSolver;
    typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

    GlobalOptimization();
    ~GlobalOptimization();

    // interface
    void optimize(int iterations=100);
    void saveOptResult(string filepath)const;

public:
    g2o::SparseOptimizer optimizer_;						// g2o optimizer
    g2o::RobustKernel* robustKernel_;						// robust kernel
    double optimizationCostTime_;							// optimization cost time
};
}

#endif // GLOBALOPT_H
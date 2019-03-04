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

#include "rgbdslam/globalOpt.h"

namespace rgbdslam
{
GlobalOptimization::GlobalOptimization()
{
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );
    optimizer_.setAlgorithm( solver );
    optimizer_.setVerbose( false );

    robustKernel_=g2o::RobustKernelFactory::instance()->construct( "Cauchy" );

    optimizationCostTime_=0;
}

GlobalOptimization::~GlobalOptimization()
{

}

void GlobalOptimization::optimize(int iterations)
{
    cout<<"start optimization\n";
    boost::timer timer;
    optimizer_.initializeOptimization();
    optimizer_.optimize(iterations);
    optimizationCostTime_=timer.elapsed();
}

void GlobalOptimization::saveOptResult(string filepath)const
{
    optimizer_.save(filepath.c_str());
}
}
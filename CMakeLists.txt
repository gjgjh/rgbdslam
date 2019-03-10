cmake_minimum_required(VERSION 2.8.3)
project(rgbdslam)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

############### dependencies ######################
#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#pcl
find_package(PCL REQUIRED COMPONENT common io filters visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

#eigen
include_directories("/usr/include/eigen3")

#g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

#csparse
FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR} )

# ROS
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        cv_bridge
        message_filters
        )
include_directories(${catkin_INCLUDE_DIRS})

# DBoW3
find_package(DBoW3 REQUIRED)
include_directories(${DBoW3_LIB_DIR})

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        g2o_core g2o_stuff g2o_types_sba
        g2o_solver_csparse g2o_csparse_extension
        ${CSPARSE_LIBRARY}
        g2o_types_slam3d
        ${catkin_LIBRARIES}
        ${DBoW3_LIBS}
        )

############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/test)
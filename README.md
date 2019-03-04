# RGBD-SLAM

This program is a simple RGBD-SLAM demo. It uses RGBD images (such as Kinect images) as input, performing tracking and mapping simultaneously. I finished and improved this program based on Gao Xiang's [tutorial](http://www.cnblogs.com/gaoxiang12/) in cnblogs (in Chinese).

This SLAM program uses PnP algorithm in tracking (front end) and global Pose graph optimization in the back end. This program supports ROS (Robot Operating System) and can visualize the result (such as trajectories and dense point cloud map) in rviz. Here is a demo [video](https://www.youtube.com/watch?v=7iH6wtQ6fdY) on youtube.


# 0. Prerequisites

Ubuntu 64-bit 16.04. ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

G2O. A modified version of g2o is in Thirdparty directory, compile and install it in your computer.

# 1. Installation

Create your **catkin** workspace:

```bash
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash">> ~/.bashrc 
source ~/.bashrc
```

In your ROS package path, clone the repository:

```bash
cd ~/catkin_ws/src
git clone https://github.com/gjgjh/rgbdslam-demo
```

Compile the rgbdslam package by typing:

```bash
cd ~/catkin_ws/
catkin_make
```

# 2. Usage

Before running the program, you may need to adjust some parameters setting in config/default.yaml. For detailed information, you can look into the comment.

After that, simply launch the launch file in the terminal:

```bash
roslaunch rgbdslam rgbdslam.launch
```

Download the rgbd dataset from TUM website: https://vision.in.tum.de/data/datasets/rgbd-dataset Then, you can play the rgbd dataset by typing these in terminal, for example:

```bash
rosbag play YOUR_DATA_FOLDER/rgbd_dataset_freiburg1_desk.bag
```

You will see two windows showing the current rgb and depth frame, and another window showing the 3D map and trajectories in real time. Before you shutdown the node, you can save the final 3D map (in .pcd format) by typing these in a new terminal:
```bash
roslaunch rgbdslam map_saver.launch
```

You can use pcl_viewer to view this map. After all data is done and your map is saved, you can press ctrl+C to stop these nodes. Some other results like trajectories (in .txt format) are also saved in the result directory. If you want to evaluate the SLAM results, there are some tools in the /scripts directory for the benchmark. The use is like:

```bash
python2 scripts/evaluate_ate.py scripts/rgbd_dataset_freiburg1_desk-groundtruth.txt result/CameraPoses.txt --plot figure.png --offset 0 --scale 1 --verbose
```

You can also use the [online evaluation](https://vision.in.tum.de/data/datasets/rgbd-dataset/online_evaluation) provided by TUM website. Don't worry if their coordiate systems don't match, the true and the estimated trajectory will be aligned beforing evaluating.

<img src="https://github.com/gjgjh/rgbdslam-demo/blob/master/support_files/figure.png" width = 50% height = 50% />

# 3. License

The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues or commercial inquiries, please contact GJH <guojh_rs@pku.edu.cn>.

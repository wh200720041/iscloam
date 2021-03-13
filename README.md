# ISCLOAM 
## Intensity Scan Context based Full SLAM Implementation (ISC-LOAM) 

This work is an implementation of paper "Intensity Scan Context: Coding Intensity and Geometry Relations for Loop Closure Detection" in IEEE International Conference on Robotics and Automation 2020 (ICRA) [paper](https://arxiv.org/pdf/2003.05656.pdf)
This work is 3D lidar based Simultaneous Localization And Mapping (SLAM), including both front-end and back-end SLAM, at 20Hz. 

**Author:** [Wang Han](http://wanghan.pro), Nanyang Technological University, Singapore

For front-end only odometry, you may visit [FLOAM (fast lidar odometry and mapping)](https://github.com/wh200720041/floam)
## 1. Evaluation
### 1.1. Demo
Watch our demo at [Video Link](https://youtu.be/Kfi6CFK4Ke4)
<p align='center'>
<a href="https://youtu.be/0-plzzxibHA" target="_blank">
<img width="65%" src="/img/iscloam_title.gif"/>
</a>
</p>

### 1.2. Mapping Example
<p align='center'>
<a href="https://youtu.be/0-plzzxibHA" target="_blank">
<img width="65%" src="/img/iscloam_kitti_mapping.gif"/>
</a>
</p>

### 1.3. Localization Example
<p align='center'>
<img width="65%" src="/img/iscloam_kitti.gif"/>
</p>


### 1.4. Ground Truth Comparison
Green: ISCLOAM        Red: Ground Truth
<p align='center'>
<img src="/img/00.png" width = 45% />
<img src="/img/05.png" width = 45% />
</p>

                      KITTI sequence 00                                  KITTI sequence 05

### 1.5. Localization error  

Platform: Intel® Core™ i7-8700 CPU @ 3.20GHz 

Average translation error : 1.08%

Average rotation error : 0.000073

### 1.6. Comparison
| Dataset                                      | ISCLOAM                    | FLOAM                  |
|----------------------------------------------|----------------------------|------------------------|
| `KITTI sequence 00`                          | 0.24%                      | 0.51%                  |
| `KITTI sequence 05`                          | 0.22%                      | 0.93%                  |

## 2. Prerequisites
### 2.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.

ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 2.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

### 2.3. **GTSAM**
Follow [GTSAM Installation](https://gtsam.org/get_started/).

### 2.3. **OPENCV**
Follow [OPENCV Installation](https://opencv.org/releases/).

### 2.4. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-melodic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

## 3. Build 
### 3.1 Clone repository:
```
cd ~/catkin_ws/src
git clone https://github.com/wh200720041/iscloam.git
cd ..
catkin_make -j1
source ~/catkin_ws/devel/setup.bash
```


### 3.2 Download test rosbag
Download [KITTI sequence 05](https://drive.google.com/file/d/1eyO0Io3lX2z-yYsfGHawMKZa5Z0uYJ0W/view?usp=sharing) (10GB) or [KITTI sequence 07](https://drive.google.com/file/d/1_qUfwUw88rEKitUpt1kjswv7Cv4GPs0b/view?usp=sharing) (4GB)

Unzip compressed file 2011_09_30_0018.zip. If your system does not have unzip. please install unzip by 
```
sudo apt-get install unzip 
```

This may take a few minutes to unzip the file, by default the file location should be /home/user/Downloads/2011_09_30_0018.bag
```
cd ~/Downloads
unzip ~/Downloads/2011_09_30_0018.zip
```

### 3.3 Launch ROS
```
roslaunch iscloam iscloam.launch
```

### 3.4 Mapping Node
if you would like to generate the map of environment at the same time, you can run 
```
roslaunch iscloam iscloam_mapping.launch
```
Note that the global map can be very large, so it may takes a while to perform global optimization, some lag is expected between trajectory and map since they are running in separate thread. More CPU usage will happen when loop closure is identified.

## 4. Test other sequence
To generate rosbag file of kitti dataset, you may use the tools provided by 
[kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) or [kitti2bag](https://github.com/tomas789/kitti2bag) 

## 5. Other Velodyne sensor
You may use iscloam_velodyne.launch for your own velodyne sensor, such as Velodyne VLP-16. 


## 6. Citation
If you use this work for your research, you may want to cite the paper below, your citation will be appreciated 
```
@inproceedings{wang2020intensity,
  author={H. {Wang} and C. {Wang} and L. {Xie}},
  booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Intensity Scan Context: Coding Intensity and Geometry Relations for Loop Closure Detection}, 
  year={2020},
  volume={},
  number={},
  pages={2095-2101},
  doi={10.1109/ICRA40945.2020.9196764}
}
```

## 7.Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).



# ISCLOAM 
## Intensity Scan Context based Full SLAM Implementation (ISC-LOAM) 

This work is 3D lidar based Simultaneous Localization And Mapping (SLAM), including both front-end and back-end SLAM. 

**Author:** [Wang Han](http://wanghan.pro), Nanyang Technological University, Singapore

## 1. Evaluation
### 1.1. Example
<p align='center'>
<img src="https://github.com/wh200720041/iscloam/blob/master/img/iscloam_kitti.gif"/>
</p>

### 1.2. Ground Truth Comparison
<p align='center'>
<img src="https://github.com/wh200720041/iscloam/blob/master/img/KITTI00.png" width = 45% />
<img src="https://github.com/wh200720041/iscloam/blob/master/img/KITTI05.png" width = 45% />
</p>

                      KITTI sequence 00                                  KITTI sequence 05

### 1.3. Localization error
Platform: Intel® Core™ i7-8700 CPU @ 3.20GHz 
<p align='center'>
| Dataset                                      | ISCLOAM                    | FLOAM                  |
|----------------------------------------------|----------------------------|------------------------|
| `KITTI sequence 00`                          | 0.24%                      | 0.51%                  |
| `KITTI sequence 05`                          | 0.22%                      | 0.93%                  |
</p>

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

## 3. Build 
### 3.1 Clone repository:
```
cd ~/catkin_ws/src
git clone https://github.com/wh200720041/iscloam.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```
### 3.2 Download test rosbag
Download [KITTI sequence 05](https://drive.google.com/open?id=18ilF7GZDg2tmT6sD5pd1RjqO0XJLn9Mv) or [KITTI sequence 07](https://drive.google.com/open?id=1VpoKm7f4es4ISQ-psp4CV3iylcA4eu0-)

Unzip compressed file 2011_09_30_0018.zip. If your system does not have unzip. please install unzip by 
```
sudo apt-get install unzip 
```

And then copy the file 2011_09_30_0018.bag into ~/catkin_ws/src/iscloam/dataset/ (this may take a few minutes to unzip the file)
```
cd ~/catkin_ws/src/iscloam/dataset/
unzip ~/Downloads/2011_09_30_0018.zip
```

### 3.3 Launch ROS
```
roslaunch iscloam iscloam.launch
```

## 4. Test other sequence
To generate rosbag file of kitti dataset, you may use the tools provided by 
[kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) or [kitti2bag](https://github.com/tomas789/kitti2bag) 

## 5. Citation
If you use this work for your research, please cite
```
@article{wang2020intensity,
  title={Intensity Scan Context: Coding Intensity and Geometry Relations for Loop Closure Detection},
  author={Wang, Han and Wang, Chen and Xie, Lihua},
  journal={arXiv preprint arXiv:2003.05656},
  year={2020}
}
```

## 6.Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).


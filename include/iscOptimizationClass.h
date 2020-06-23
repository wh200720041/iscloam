// Author of ISCLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef _ISC_OPTIMIZATION_CLASS_H_
#define _ISC_OPTIMIZATION_CLASS_H_

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/ndt.h>

//GTSAM
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

//ros
#include <ros/ros.h>

//local lib
#include "lidarOptimization.h"

#define LOOPCLOSURE_THRESHOLD 41
//stop loop check for the next N frames if loop is identified
#define STOP_LOOP_CHECK_COUNTER 40
class ISCOptimizationClass
{
    public:
        ISCOptimizationClass();

        void init(void);

        //return true if global optimized
        bool addPoseToGraph(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud_edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud_surf_in, std::vector<int>& matched_frame_id, Eigen::Isometry3d& odom_in);

        Eigen::Isometry3d getLastPose(void);

        Eigen::Isometry3d getPose(int frame_num);

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointcloud_arr;

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointcloud_surf_arr;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointcloud_edge_arr;

        pcl::PointCloud<pcl::PointXYZI>::Ptr loop_candidate_pc = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()); 
        pcl::PointCloud<pcl::PointXYZI>::Ptr loop_map_pc = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()); 
        pcl::PointCloud<pcl::PointXYZI>::Ptr map = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()); 
        
    private:
        std::vector<gtsam::Pose3> pose_optimized_arr;
        std::vector<gtsam::Pose3> odom_original_arr;
        gtsam::Pose3 last_pose3;
        
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initials;

        gtsam::noiseModel::Diagonal::shared_ptr priorModel;

        gtsam::noiseModel::Diagonal::shared_ptr odomModel;

        gtsam::noiseModel::Diagonal::shared_ptr loopModel;

        int stop_check_loop_count = 0;

        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
        
        Eigen::Isometry3d pose3ToEigen(const gtsam::Pose3& pose3);

        gtsam::Pose3 eigenToPose3(const Eigen::Isometry3d& pose_eigen);

        void globalOptimization(void);

        bool geometryConsistencyVerification(int current_id, int matched_id, Eigen::Isometry3d& transform);

        bool updateStates(gtsam::Values& result, int matched_id, int current_id);
     
        double estimateOdom(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_source_edge, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_source_surf, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_target_edge, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_target_surf, Eigen::Isometry3d& transform);

};


#endif // _ISC_OPTIMIZATION_CLASS_H_ 


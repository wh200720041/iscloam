// Author of ISCLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef _ISC_GENERATION_CLASS_H_
#define _ISC_GENERATION_CLASS_H_

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>


//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//ros
#include <ros/ros.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>


//IF TRAVELLED DISTANCE IS LESS THAN THIS VALUE, SKIP FOR PLACE RECOGNTION
#define SKIP_NEIBOUR_DISTANCE 20.0
//how much error will odom generate per frame 
#define INFLATION_COVARIANCE 0.03

//define threshold for loop closure detection
#define GEOMETRY_THRESHOLD 0.67
#define INTENSITY_THRESHOLD 0.91

typedef cv::Mat ISCDescriptor; 


class ISCGenerationClass
{
    public:
        ISCGenerationClass();
        void init_param(int rings_in, int sectors_in, double max_dis_in);

        ISCDescriptor getLastISCMONO(void);
        ISCDescriptor getLastISCRGB(void);
        void loopDetection(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_pc, Eigen::Isometry3d& odom);
        
        int current_frame_id;
        std::vector<int> matched_frame_id;
    private:
        int rings = 20;
        int sectors = 90;
        double ring_step=0.0;
        double sector_step=0.0;
        double max_dis = 60; 

        std::vector<cv::Vec3b> color_projection;

        
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_point_cloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr test_pc;

        std::vector<Eigen::Vector3d> pos_arr;
        std::vector<double> travel_distance_arr;
        std::vector<ISCDescriptor> isc_arr;

        void init_color(void);
        void print_param(void);
        bool is_loop_pair(ISCDescriptor& desc1, ISCDescriptor& desc2, double& geo_score, double& inten_score);
        ISCDescriptor calculate_isc(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pointcloud);
        double calculate_geometry_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle);
        double calculate_intensity_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle);
        void ground_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out);
};




#endif // _ISC_GENERATION_CLASS_H_


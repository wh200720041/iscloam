// Author of ISCLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "laserMappingClass.h"
#include "lidar.h"

LaserMappingClass laserMapping;
lidar::Lidar lidar_param;
std::mutex mutex_lock;
std::queue<nav_msgs::PathConstPtr> pathBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

ros::Publisher map_pub;
void pathHandler(const nav_msgs::PathConstPtr &pathMsg)
{
    mutex_lock.lock();
    pathBuf.push(pathMsg);
    mutex_lock.unlock();
}

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

Eigen::Isometry3d geometryToEigen(geometry_msgs::PoseStamped& pose_geo){
    Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
    pose_eigen.rotate(Eigen::Quaterniond(pose_geo.pose.orientation.w,pose_geo.pose.orientation.x,pose_geo.pose.orientation.y,pose_geo.pose.orientation.z));  
    pose_eigen.pretranslate(Eigen::Vector3d(pose_geo.pose.position.x,pose_geo.pose.position.y,pose_geo.pose.position.z));
    return pose_eigen;
}

void laser_mapping(){
    while(1){
        if(!pathBuf.empty() && !pointCloudBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.toSec()<pathBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
                ROS_WARN("time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node"); 
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;              
            }

            if(!pathBuf.empty() && pathBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
                pathBuf.pop();
                ROS_INFO("time stamp unaligned with path final, pls check your data --> laser mapping node");
                mutex_lock.unlock();
                continue;  
            }

            //if time aligned 
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            nav_msgs::Path path = *(pathBuf.front());
            pointCloudBuf.pop();
            pathBuf.pop();
            mutex_lock.unlock();
            
            int path_size = path.poses.size();
            if(path.poses.size()!=laserMapping.point_cloud_arr.size()+1){
                ROS_WARN("path size %d and point cloud size %d not aligned, laser mapping", path_size, laserMapping.point_cloud_arr.size()+1);
            }

            //check is the path deviates from last path by checking the last odom
            if(path.poses.size()>2){
                Eigen::Isometry3d pose_temp = geometryToEigen(path.poses[path_size-2]);
                Eigen::Isometry3d pose_error = laserMapping.last_pose * pose_temp.inverse();

                double translational_error = abs(pose_error.translation().x()) + abs(pose_error.translation().y()) + abs(pose_error.translation().z());
                if(translational_error>0.1){
                    //if global optimization is applied 
                    std::vector<Eigen::Isometry3d> path_eigen;
                    for(int i=0;i<path_size-2;i++){
                        path_eigen.push_back(geometryToEigen(path.poses[i]));
                    }
                    laserMapping.resetMap(path_eigen);
                }
            }
            laserMapping.updateCurrentPointsToMap(pointcloud_in,geometryToEigen(path.poses[path_size-1]));

            // Eigen::Isometry3d curr = geometryToEigen(path.poses[path_size-1]);
            // Eigen::Quaterniond q_curr(curr.linear());
            // static tf::TransformBroadcaster br_realtime;
            // tf::Transform transform;
            // transform.setOrigin( tf::Vector3(curr.translation().x(), curr.translation().y(), curr.translation().z()) );
            // tf::Quaternion q(q_curr.x(),q_curr.y(),q_curr.z(),q_curr.w());
            // transform.setRotation(q);
            // br_realtime.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            if(path_size %20 == 0){
                pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = laserMapping.getMap();
                sensor_msgs::PointCloud2 PointsMsg;
                pcl::toROSMsg(*pc_map, PointsMsg);
                PointsMsg.header.stamp = pointcloud_time;
                PointsMsg.header.frame_id = "map";
                map_pub.publish(PointsMsg); 
            }


        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserMapping.init(map_resolution);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100, velodyneHandler);
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<nav_msgs::Path>("/final_path", 100, pathHandler);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);
    std::thread laser_mapping_process{laser_mapping};

    ros::spin();

    return 0;
}

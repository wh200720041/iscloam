// Author of ISCLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//std lib
#include <iostream>
#include <string>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

//ros
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//opencv
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

//local library
#include "iscGenerationClass.h"
#include <iscloam/LoopInfo.h>

ros::Publisher isc_pub;
ros::Publisher loop_info_pub;


std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::mutex mutex_lock;

ISCGenerationClass iscGeneration;

double scan_period= 0.1;

//receive odomtry
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex_lock.lock();
    odometryBuf.push(msg);
    mutex_lock.unlock();

}

//receive all point cloud
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mutex_lock.lock();
    pointCloudBuf.push(msg);
    mutex_lock.unlock();
}

void loop_closure_detection(){
    while(1){
        if(!pointCloudBuf.empty() && !odometryBuf.empty()){
            //align time stamp
            mutex_lock.lock();
            if(!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec() - 0.5 * scan_period){
                ROS_WARN("isc_generation: time stamp unaligned error and odom discarded, pls check your data; odom time %f, pc time %f",odometryBuf.front()->header.stamp.toSec(),pointCloudBuf.front()->header.stamp.toSec()); 
                odometryBuf.pop();  
                mutex_lock.unlock();
                continue;
            }

            if(!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec() - 0.5 * scan_period){
                ROS_WARN("isc_generation: time stamp unaligned error and odom discarded, pls check your data; odom time %f, pc time %f",odometryBuf.front()->header.stamp.toSec(),pointCloudBuf.front()->header.stamp.toSec()); 
                pointCloudBuf.pop();  
                mutex_lock.unlock();
                continue;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            Eigen::Isometry3d odom_in = Eigen::Isometry3d::Identity();
            odom_in.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));  
            odom_in.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z));
            odometryBuf.pop();
            pointCloudBuf.pop();  
            mutex_lock.unlock();


            iscGeneration.loopDetection(pointcloud_in, odom_in);

            cv_bridge::CvImage out_msg;
            out_msg.header.frame_id  = "velodyne"; 
            out_msg.header.stamp  = pointcloud_time; 
            out_msg.encoding = sensor_msgs::image_encodings::RGB8; 
            out_msg.image    = iscGeneration.getLastISCRGB(); 
            isc_pub.publish(out_msg.toImageMsg());
            
            iscloam::LoopInfo loop;
            loop.header.stamp = pointcloud_time;
            loop.header.frame_id = "velodyne";
            loop.current_id = iscGeneration.current_frame_id;
            for(int i=0;i<(int)iscGeneration.matched_frame_id.size();i++){
                loop.matched_id.push_back(iscGeneration.matched_frame_id[i]);
            }
            loop_info_pub.publish(loop);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }//end of while(1)
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "isc_gen");

    ros::NodeHandle nh("~");

    //rosnode init
    ros::Subscriber velodyne_sub= nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 10, pointCloudCallback);
    ros::Subscriber odom_sub= nh.subscribe<nav_msgs::Odometry>("/odom", 10, odomCallback);

    loop_info_pub = nh.advertise<iscloam::LoopInfo>("/loop_closure", 100);
    isc_pub = nh.advertise<sensor_msgs::Image>("/isc", 100);


    //read parameter
    int sector_width = 60;
    int ring_height = 60;
    double max_dis= 40.0;

    nh.getParam("/sector_width", sector_width); 
    nh.getParam("/ring_height", ring_height); 
    //nh.getParam("/max_dis", max_dis); 
    nh.getParam("/scan_period", scan_period);  

    //init ISC
    iscGeneration.init_param(ring_height,sector_width,max_dis);

    //open thread
    std::thread loop_closure_detection_process{loop_closure_detection};

    ros::spin();

  return 0;
}

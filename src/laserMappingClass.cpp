// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "laserMappingClass.h"

void LaserMappingClass::init(double map_resolution_){
	//init map
	//init can have real object, but future added block does not need
	for(int i=0;i<LASER_CELL_RANGE_HORIZONTAL*2+1;i++){
		std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
		for(int j=0;j<LASER_CELL_RANGE_HORIZONTAL*2+1;j++){
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
			for(int k=0;k<LASER_CELL_RANGE_VERTICAL*2+1;k++){
				pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>);
				map_depth_temp.push_back(point_cloud_temp);	
			}
			map_height_temp.push_back(map_depth_temp);
		}
		map.push_back(map_height_temp);
	}
	map_resolution = map_resolution_;
	origin_in_map_x = LASER_CELL_RANGE_HORIZONTAL;
	origin_in_map_y = LASER_CELL_RANGE_HORIZONTAL;
	origin_in_map_z = LASER_CELL_RANGE_VERTICAL;
	map_width = LASER_CELL_RANGE_HORIZONTAL*2+1;
	map_height = LASER_CELL_RANGE_HORIZONTAL*2+1;
	map_depth = LASER_CELL_RANGE_HORIZONTAL*2+1;

	//downsampling size
	downSizeFilter.setLeafSize(map_resolution* 2.5, map_resolution* 2.5, map_resolution* 2.5);
	last_pose = Eigen::Isometry3d::Identity();
}

void LaserMappingClass::addWidthCellNegative(void){
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
	for(int j=0; j < map_height;j++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0;k< map_depth;k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map_height_temp.push_back(map_depth_temp);
	}
	map.insert(map.begin(), map_height_temp);

	origin_in_map_x++;
	map_width++;
}
void LaserMappingClass::addWidthCellPositive(void){
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
	for(int j=0; j < map_height;j++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0;k< map_depth;k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map_height_temp.push_back(map_depth_temp);
	}
	map.push_back(map_height_temp);
	map_width++;
}
void LaserMappingClass::addHeightCellNegative(void){
	for(int i=0; i < map_width;i++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0;k<map_depth;k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map[i].insert(map[i].begin(), map_depth_temp);
	}
	origin_in_map_y++;
	map_height++;
}
void LaserMappingClass::addHeightCellPositive(void){
	for(int i=0; i < map_width;i++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0;k<map_depth;k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map[i].push_back(map_depth_temp);
	}
	map_height++;
}
void LaserMappingClass::addDepthCellNegative(void){
	for(int i=0; i < map_width;i++){
		for(int j=0;j< map_height;j++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map[i][j].insert(map[i][j].begin(), point_cloud_temp);
		}
	}
	origin_in_map_z++;
	map_depth++;
}
void LaserMappingClass::addDepthCellPositive(void){
	for(int i=0; i < map_width;i++){
		for(int j=0;j< map_height;j++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map[i][j].push_back(point_cloud_temp);
		}
	}
	map_depth++;
}

//extend map is points exceed size
void LaserMappingClass::checkPoints(int& x, int& y, int& z){

	while(x + LASER_CELL_RANGE_HORIZONTAL> map_width-1){
		addWidthCellPositive();
	}
	while(x-LASER_CELL_RANGE_HORIZONTAL<0){
		addWidthCellNegative();
		x++;
	}
	while(y + LASER_CELL_RANGE_HORIZONTAL> map_height-1){
		addHeightCellPositive();
	}
	while(y-LASER_CELL_RANGE_HORIZONTAL<0){
		addHeightCellNegative();
		y++;
	}
	while(z + LASER_CELL_RANGE_VERTICAL> map_depth-1){
		addDepthCellPositive();
	}
	while(z-LASER_CELL_RANGE_VERTICAL<0){
		addDepthCellNegative();
		z++;
	}

	//initialize 
	//create object if area is null
	for(int i=x-LASER_CELL_RANGE_HORIZONTAL;i<x+LASER_CELL_RANGE_HORIZONTAL+1;i++){
		for(int j=y-LASER_CELL_RANGE_HORIZONTAL;j<y+LASER_CELL_RANGE_HORIZONTAL+1;j++){
			for(int k=z-LASER_CELL_RANGE_VERTICAL;k<z+LASER_CELL_RANGE_VERTICAL+1;k++){
				if(map[i][j][k]==NULL){
					pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
					map[i][j][k] = point_cloud_temp;
				}

			}
				
		}

	}
}

void LaserMappingClass::resetMap(std::vector<Eigen::Isometry3d>& path){
	map.clear();
	init(map_resolution);

	for(int i=0;i<path.size();i++){
		int currentPosIdX = int(std::floor(path[i].translation().x() / LASER_CELL_WIDTH + 0.5)) + origin_in_map_x;
		int currentPosIdY = int(std::floor(path[i].translation().y() / LASER_CELL_HEIGHT + 0.5)) + origin_in_map_y;
		int currentPosIdZ = int(std::floor(path[i].translation().z() / LASER_CELL_DEPTH + 0.5)) + origin_in_map_z;

		//check is submap is null
		checkPoints(currentPosIdX,currentPosIdY,currentPosIdZ);

		pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::transformPointCloud(*point_cloud_arr[i], *transformed_pc, path[i].cast<float>());
		
		
		for (int j = 0; j < (int)transformed_pc->points.size(); j++)
		{
			pcl::PointXYZI point_temp = transformed_pc->points[j];
			//for visualization only
			//point_temp.intensity = std::min(1.0 , std::max(point_cloud_arr[i]->points[j].z+2.0, 0.0) / 4);

			int currentPointIdX = int(std::floor(point_temp.x / LASER_CELL_WIDTH + 0.5)) + origin_in_map_x;
			int currentPointIdY = int(std::floor(point_temp.y / LASER_CELL_HEIGHT + 0.5)) + origin_in_map_y;
			int currentPointIdZ = int(std::floor(point_temp.z / LASER_CELL_DEPTH + 0.5)) + origin_in_map_z;

			map[currentPointIdX][currentPointIdY][currentPointIdZ]->push_back(point_temp);
			
		}
		
		//filtering points 
		for(int i=currentPosIdX-LASER_CELL_RANGE_HORIZONTAL;i<currentPosIdX+LASER_CELL_RANGE_HORIZONTAL+1;i++){
			for(int j=currentPosIdY-LASER_CELL_RANGE_HORIZONTAL;j<currentPosIdY+LASER_CELL_RANGE_HORIZONTAL+1;j++){
				for(int k=currentPosIdZ-LASER_CELL_RANGE_VERTICAL;k<currentPosIdZ+LASER_CELL_RANGE_VERTICAL+1;k++){
					downSizeFilter.setInputCloud(map[i][j][k]);
					downSizeFilter.filter(*(map[i][j][k]));
				}
					
			}

		}

	}

	last_pose = path[path.size()-1];
}

//update points to map 
void LaserMappingClass::updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const Eigen::Isometry3d& pose_current){
	
	int currentPosIdX = int(std::floor(pose_current.translation().x() / LASER_CELL_WIDTH + 0.5)) + origin_in_map_x;
	int currentPosIdY = int(std::floor(pose_current.translation().y() / LASER_CELL_HEIGHT + 0.5)) + origin_in_map_y;
	int currentPosIdZ = int(std::floor(pose_current.translation().z() / LASER_CELL_DEPTH + 0.5)) + origin_in_map_z;

	//check is submap is null
	checkPoints(currentPosIdX,currentPosIdY,currentPosIdZ);

	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::transformPointCloud(*pc_in, *transformed_pc, pose_current.cast<float>());
	
	//储存点
	for (int i = 0; i < (int)transformed_pc->points.size(); i++)
	{
		pcl::PointXYZI point_temp = transformed_pc->points[i];
		//for visualization only
		//point_temp.intensity = std::min(1.0 , std::max(pc_in->points[i].z+2.0, 0.0) / 4);
		
		int currentPointIdX = int(std::floor(point_temp.x / LASER_CELL_WIDTH + 0.5)) + origin_in_map_x;
		int currentPointIdY = int(std::floor(point_temp.y / LASER_CELL_HEIGHT + 0.5)) + origin_in_map_y;
		int currentPointIdZ = int(std::floor(point_temp.z / LASER_CELL_DEPTH + 0.5)) + origin_in_map_z;

		map[currentPointIdX][currentPointIdY][currentPointIdZ]->push_back(point_temp);
		
	}
	
	//filtering points 
	for(int i=currentPosIdX-LASER_CELL_RANGE_HORIZONTAL;i<currentPosIdX+LASER_CELL_RANGE_HORIZONTAL+1;i++){
		for(int j=currentPosIdY-LASER_CELL_RANGE_HORIZONTAL;j<currentPosIdY+LASER_CELL_RANGE_HORIZONTAL+1;j++){
			for(int k=currentPosIdZ-LASER_CELL_RANGE_VERTICAL;k<currentPosIdZ+LASER_CELL_RANGE_VERTICAL+1;k++){
				downSizeFilter.setInputCloud(map[i][j][k]);
				downSizeFilter.filter(*(map[i][j][k]));
			}
				
		}

	}
	point_cloud_arr.push_back(pc_in);
	last_pose = pose_current;
	//publish points every 20 frames
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LaserMappingClass::getMap(void){
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new  pcl::PointCloud<pcl::PointXYZI>());
	for (int i = 0; i < map_width; i++){
		for (int j = 0; j < map_height; j++){
			for (int k = 0; k < map_depth; k++){
				if(map[i][j][k]!=NULL){
					*laserCloudMap += *(map[i][j][k]);
				}
			}
		}
	}
	return laserCloudMap;
}

LaserMappingClass::LaserMappingClass(){

}

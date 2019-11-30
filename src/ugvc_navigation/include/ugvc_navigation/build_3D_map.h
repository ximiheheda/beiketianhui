
#include "ros/ros.h"
#include "std_msgs/String.h"
#include<pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <sstream>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <sstream>
#include <fstream>
#include <ros/time.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Odometry.h>
#include <ugvc_navigation/mymap.h>
#include <ugvc_navigation/front_obstacle.h>


double globalMapRes = 0.2;
int globalMapWidth = 100;                         // Global image
int globalMapLongth = 100;
int globalMapHeight = 16;
//把格子中心扔到左下角
double globalMapX0 = -globalMapLongth*globalMapRes/2;
double globalMapY0 = -globalMapWidth*globalMapRes/2;
double globalMapZ0 = -globalMapHeight*globalMapRes/2;
//double globalMapX0 = 0;
//double globalMapY0 = 0;
//double globalMapZ0 = 0;


double inflation_r = 1.2f;
double inflation_r_lethal = 0.4f;
double inflation_r_circle = 2.0f;

//存储各种索引
int uu_map = 0, vv_map = 0, ww_map = 0;
int uu_uav = 0, vv_uav = 0, ww_uav = 0;
int uu_front = 0, vv_front = 0, ww_front = 0;

// const double localMapRes = 0.2;
// const int globalMapWidth = 100;                         // Local image
// const int localMapHeight = 100;
// int a=globalMapHeight*globalMapWidth;

//存储迷宫的栅格地图
ugvc_navigation::mymap mazemap;
//存储与无人机同一高度的栅格地图，分为左墙和右墙
//ugvc_navigation::front_obstacle obstacle_front_map_left;
//ugvc_navigation::front_obstacle obstacle_front_map_right;
ugvc_navigation::front_obstacle obstacle_front_map;
ugvc_navigation::front_obstacle window_front_map;
//存储无人机所在位置的栅格地图
ugvc_navigation::mymap uav_map;

int pArray[160000]={0};//长乘宽乘高 uu+vv*globalMapWidth+ww*globalMapLongth*globalMapLongth;
int lvbo[160000]={0};
// int pArray[globalMapHeight*globalMapWidth]={};
const double HisRate = 0.7;     

nav_msgs::GridCells maze_cell_map;//maze cell map
nav_msgs::GridCells uav_cell_map; //uav cell map
nav_msgs::GridCells front_cell_map; //front cell map
nav_msgs::GridCells window_front_cell_map; //window front cell map
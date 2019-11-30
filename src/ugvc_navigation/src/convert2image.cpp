
#include <ugvc_navigation/build_3D_map.h>
using namespace std;
/**
 * 本函数的作用：利用激光雷达的点云数据，分别得到迷宫栅格地图和无人机的栅格地图
 */
class convert2image
{
  private:
  ros::NodeHandle nh;	
  ros::Subscriber map_sub;
  ros::Publisher map_pub;
  ros::Subscriber uav_sub;
  ros::Publisher uav_pub;
  ros::Subscriber front_sub;
  ros::Publisher front_pub;
  ros::Subscriber window_front_sub;
  ros::Publisher window_front_pub;

  public:
//开始定义地图
                        // Global image

#define LM_X2V(x) (int)(-(globalMapX0-(x))/globalMapRes)
#define LM_Y2U(y) (int)(-(globalMapY0-(y))/globalMapRes)
#define LM_Z2W(z) (int)(-(globalMapZ0-(z))/globalMapRes)
#define LM_V2X(v) (globalMapX0+(v)*globalMapRes)
#define LM_U2Y(u) (globalMapY0+(u)*globalMapRes)
#define LM_W2Z(w) (globalMapZ0+(w)*globalMapRes)
tf::TransformListener plistener;
// int mapsize=(int)(globalMapLongth*globalMapWidth*globalMapHeight);
// int pArray[mapsize]={};

    convert2image()
    {
        map_sub = nh.subscribe <ugvc_navigation::mymap> ("/cmap", 1, &convert2image::maze_cloud_cb,this);
        map_pub = nh.advertise <nav_msgs::GridCells> ("/maze_map", 10);

        uav_sub = nh.subscribe <ugvc_navigation::mymap> ("/uavmap", 1, &convert2image::uav_cloud_cb,this);
        uav_pub = nh.advertise <nav_msgs::GridCells> ("/uav_map", 10);

        front_sub = nh.subscribe <ugvc_navigation::mymap> ("/frontmap", 1, &convert2image::front_cloud_cb,this);
        front_pub = nh.advertise <nav_msgs::GridCells> ("/front_map", 10);

        window_front_sub = nh.subscribe <ugvc_navigation::mymap> ("/windowfrontmap", 1, &convert2image::window_front_cloud_cb,this);
        window_front_pub = nh.advertise <nav_msgs::GridCells> ("/window_map", 10);
    }
	
	
    void maze_cloud_cb (const ugvc_navigation::mymap::ConstPtr& map_data1) 
    {
        maze_cell_map.header.frame_id="odom";
        maze_cell_map.header.stamp = ros::Time::now();
        maze_cell_map.cell_height=0.2;
        maze_cell_map.cell_width=0.2;
        //maze_cell_map.cells.clear();
        maze_cell_map.cells.resize(160000);
        geometry_msgs::PointStamped laser_point;
        
        laser_point.header.frame_id="base_link";
        
        geometry_msgs::PointStamped base_point; 
        base_point.header.frame_id = "odom"; 

        for(size_t i=0;i<160000;i++)
        {
            if( map_data1->maparry[i]==1)
            {
                geometry_msgs::Point obstacle;  
                obstacle.x = LM_V2X(floor((i%(globalMapLongth*globalMapLongth))/globalMapWidth));
                obstacle.y = LM_U2Y((i%(globalMapLongth*globalMapLongth))%globalMapWidth);
                obstacle.z = LM_W2Z((i/(globalMapLongth*globalMapLongth))); 
                // ROS_INFO("x=%lfy=%lfz=%lf",obstacle.x,obstacle.y,obstacle.z);
                maze_cell_map.cells.push_back(obstacle);
                // ROS_INFO("x=%lf,y=%lf,z=%lf",obstacle.x,obstacle.y,obstacle.z);   
            }
        
        }
        map_pub.publish(maze_cell_map);
    }

    void uav_cloud_cb (const ugvc_navigation::mymap::ConstPtr& map_data2) 
	{
        uav_cell_map.header.frame_id="odom";
        uav_cell_map.header.stamp = ros::Time::now();
        uav_cell_map.cell_height=0.2;
        uav_cell_map.cell_width=0.2;
        uav_cell_map.cells.resize(160000);
        geometry_msgs::PointStamped laser_point;
        
        laser_point.header.frame_id="base_link";
        
        geometry_msgs::PointStamped base_point; 
        base_point.header.frame_id = "odom"; 

        for(size_t i=0;i<160000;i++)
        {      
            if( map_data2->maparry[i]==1)
            {
                geometry_msgs::Point obstacle;  
                obstacle.x = LM_V2X(floor((i%(globalMapLongth*globalMapLongth))/globalMapWidth));
                obstacle.y = LM_U2Y((i%(globalMapLongth*globalMapLongth))%globalMapWidth);
                obstacle.z = LM_W2Z((i/(globalMapLongth*globalMapLongth))); 

                uav_cell_map.cells.push_back(obstacle);
            }
        
        }
        uav_pub.publish(uav_cell_map);
	}

    void front_cloud_cb (const ugvc_navigation::mymap::ConstPtr& map_data3) 
	{
        front_cell_map.header.frame_id="odom";
        front_cell_map.header.stamp = ros::Time::now();
        front_cell_map.cell_height=0.2;
        front_cell_map.cell_width=0.2;
        front_cell_map.cells.resize(160000);
        geometry_msgs::PointStamped laser_point;
        
        laser_point.header.frame_id="base_link";
        
        geometry_msgs::PointStamped base_point; 
        base_point.header.frame_id = "odom"; 

        for(size_t i=0;i<160000;i++)
        {      
            if( map_data3->maparry[i]==1)
            {
                geometry_msgs::Point obstacle;  
                obstacle.x = LM_V2X(floor((i%(globalMapLongth*globalMapLongth))/globalMapWidth));
                obstacle.y = LM_U2Y((i%(globalMapLongth*globalMapLongth))%globalMapWidth);
                obstacle.z = LM_W2Z((i/(globalMapLongth*globalMapLongth))); 

                front_cell_map.cells.push_back(obstacle);
            }
        
        }
        front_pub.publish(front_cell_map);
	}

    void window_front_cloud_cb (const ugvc_navigation::mymap::ConstPtr& map_data) 
	{
        window_front_cell_map.header.frame_id="odom";
        window_front_cell_map.header.stamp = ros::Time::now();
        window_front_cell_map.cell_height=0.2;
        window_front_cell_map.cell_width=0.2;
        window_front_cell_map.cells.resize(160000);
        geometry_msgs::PointStamped laser_point;
        
        laser_point.header.frame_id="base_link";
        
        geometry_msgs::PointStamped base_point; 
        base_point.header.frame_id = "odom"; 

        for(size_t i=0;i<160000;i++)
        {      
            if( map_data->maparry[i]==1)
            {
                geometry_msgs::Point obstacle;  
                obstacle.x = LM_V2X(floor((i%(globalMapLongth*globalMapLongth))/globalMapWidth));
                obstacle.y = LM_U2Y((i%(globalMapLongth*globalMapLongth))%globalMapWidth);
                obstacle.z = LM_W2Z((i/(globalMapLongth*globalMapLongth))); 

                window_front_cell_map.cells.push_back(obstacle);
            }
        
        }
        window_front_pub.publish(window_front_cell_map);
	}
};

int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "convert2image");
    convert2image ic;
	
	ros::spin();
	return 0;
}

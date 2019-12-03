
#include <ugvc_navigation/build_3D_map.h>

#define LM_X2V(x) (int)((-globalMapX0+(x))/globalMapRes)
#define LM_Y2U(y) (int)((-globalMapY0+(y))/globalMapRes)
#define LM_Z2W(z) (int)((-globalMapZ0+(z))/globalMapRes)
#define LM_V2X(v) (globalMapX0+(v)*globalMapRes)
#define LM_U2Y(u) (globalMapY0+(u)*globalMapRes)
#define LM_W2Z(w) (globalMapY0+(w)*globalMapRes)

using namespace std;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
class build_3D_map
{
  private:
  ros::NodeHandle nh;	
  ros::Subscriber points_sub;
  ros::Subscriber uav_locat_sub;
  ros::Publisher map_pub;
  ros::Publisher uav_pub;
  //ros::Publisher obstacle_front_left_pub;
  //ros::Publisher obstacle_front_right_pub;
  ros::Publisher obstacle_front_pub;
  ros::Publisher window_front_pub;
  public:

  // Location of UAV
  geometry_msgs::PointStamped uav_location;

  //call back functions
  void uav_loc_cb(const nav_msgs::Odometry::ConstPtr& uav_locat_msg);
	void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

  //开始定义地图
  // Global image
  build_3D_map();

  tf::TransformListener plistener;
  // int mapsize=(int)(globalMapLongth*globalMapWidth*globalMapHeight);
  // int pArray[mapsize]={};

};

build_3D_map::build_3D_map()
{
  points_sub = nh.subscribe <sensor_msgs::PointCloud2> ("/rfans_driver/rfans_points", 1, &build_3D_map::cloud_cb,this);
  uav_locat_sub = nh.subscribe <nav_msgs::Odometry> ("/odom", 1, &build_3D_map::uav_loc_cb,this);

  map_pub = nh.advertise<ugvc_navigation::mymap> ("/cmap", 10); 
  uav_pub = nh.advertise<ugvc_navigation::mymap> ("/uavmap", 10);
  //obstacle_front_left_pub = nh.advertise<ugvc_navigation::mymap> ("/front_left_map", 10);
  //obstacle_front_right_pub = nh.advertise<ugvc_navigation::mymap> ("/front_left_map", 10);
  obstacle_front_pub = nh.advertise<ugvc_navigation::mymap> ("/frontmap", 10);
  window_front_pub = nh.advertise<ugvc_navigation::mymap> ("/windowfrontmap",10);
  
  while(ros::ok())
  {
    ros::Rate rate(10.0); 
    ros::spinOnce();
    rate.sleep();
  }
}
  
void build_3D_map::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) 
{
  // C0-update
  tf::StampedTransform transform;
  try{
    //  ROS_INFO("SSS");
      plistener.waitForTransform("/odom", "/base_link",
                                  ros::Time::now(), ros::Duration(10));
      plistener.lookupTransform("/odom", "/base_link",
                                  ros::Time(0), transform);   // Global image
    
  }
  catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  mazemap.header.frame_id="odom";
  mazemap.header.stamp = ros::Time::now();
  geometry_msgs::PointStamped laser_point;
  
  laser_point.header.frame_id="base_link";
  
  geometry_msgs::PointStamped base_point; 
  base_point.header.frame_id = "odom"; 
  /*------------------------------------将栅格地图mazemap,uavmap,frontmap清零，即每次均遗忘---------------------------------------*/
  for(size_t i=0;i<160000;i++)
  {
    uav_map.maparry[i] = 0;
    mazemap.maparry[i] = 0;
    //obstacle_front_map_left.maparry[i] = 0;
    //obstacle_front_map_right.maparry[i] = 0;
    obstacle_front_map.maparry[i] = 0;
    window_front_map.maparry[i] = 0;
  }


  /*------------------------------------计算无人机的栅格地图---------------------------------------*/
  // 计算odom坐标系里面，uav所在的索引位置
  if(abs(uav_location.point.x)<10&&abs(uav_location.point.y)<10&&abs(uav_location.point.z)<1.5)
  {
    vv_uav=LM_X2V(uav_location.point.x);
    uu_uav=LM_Y2U(uav_location.point.y);
    ww_uav=LM_Z2W(uav_location.point.z);
    //将无人机的坐标添加无人机栅格地图
    uav_map.maparry[uu_uav+vv_uav*globalMapWidth+ww_uav*globalMapLongth*globalMapLongth] = 1;
    //mymap.maparry[uu+vv*globalMapWidth+ww*globalMapLongth*globalMapLongth]=1;
  }



  /*------------------------------------计算迷宫栅格地图----------------------------------------------*/

  for(size_t i=0;i<cloud.size();i++)
  {
    laser_point.point.x=cloud[i].x;
    laser_point.point.y=cloud[i].y;
    laser_point.point.z=cloud[i].z;

    plistener.transformPoint("odom", ros::Time(0),
                                      laser_point, "odom", base_point);

    if(abs(base_point.point.x)<10&&abs(base_point.point.y)<10&&abs(base_point.point.z)<1.5)//&&vv>=0&&uu>=0&&ww>=0)
    {
      vv_map = LM_X2V(base_point.point.x);
      uu_map = LM_Y2U(base_point.point.y);
      ww_map = LM_Z2W(base_point.point.z);
      // ROS_INFO("vv=%d,uu=%d,ww=%d",vv,uu,ww);
      // 按照x的方向开始横向扫描，扫满以后再向y方向前进一格，扫满整个平面以后，再向z方向上升一格

      lvbo[uu_map+vv_map*globalMapWidth+ww_map*globalMapLongth*globalMapLongth]=lvbo[uu_map+vv_map*globalMapWidth+ww_map*globalMapLongth*globalMapLongth]+50;
      // 减去部分没有加
      if(lvbo[uu_map+vv_map*globalMapWidth+ww_map*globalMapLongth*globalMapLongth]>=100)
      {
        mazemap.maparry[uu_map+vv_map*globalMapWidth+ww_map*globalMapLongth*globalMapLongth]=1;
        lvbo[uu_map+vv_map*globalMapWidth+ww_map*globalMapLongth*globalMapLongth]=100;
      }
     }
    }
    //发布迷宫栅格地图
    map_pub.publish(mazemap);
    //发布无人机所在位置栅格地图
    uav_pub.publish(uav_map);

  //过滤掉最下面的一层点云（防止因为地面的干扰，从而影响障碍的判断）
  //先不过滤了，这样的话飞机起飞的时候，万一侦查不到地面，则可能导致一些墙的信息
  /*size_t lowest_plane_index = -1;
  for(size_t i = 0; i<globalMapHeight; i++)
  {
    for(size_t j = 0; j<globalMapLongth; j++)
    {
      for(size_t k = 0; k<globalMapWidth; k++)
      {
        if(mazemap.maparry[k+j*globalMapWidth+i*globalMapLongth*globalMapLongth] == 1)
        {
          lowest_plane_index = i;
        }
      }yuejie
    }
    if(int(lowest_plane_index) > -1)
    {
      for(size_t j = 0; j<globalMapLongth; j++)
      {
        for(size_t k = 0; k<globalMapWidth; k++)
        {
          mazemap.maparry[k+j*globalMapWidth+i*globalMapLongth*globalMapLongth] = 0;
        }
      }
      break;
    }
  }
  ROS_INFO("The lowest plane_index is %d", int(lowest_plane_index));*/
  /*------------------------------------计算与无人机高度面前的墙的栅格地图---------------------------*/
  ROS_INFO("UAV: uu=%d, vv=%d, ww=%d",uu_uav, vv_uav, ww_uav);
  
  size_t num_obstacle = 0; 
  size_t i=1; //y方向的索引
  size_t uu_wall_start = 0; //这一部分是计算首先无人机能够扫到的墙，然后记录该墙所在的uu值，将该墙的uu的值的前后3格（60厘米）的值全部累加到当前的uu
  //若仍然为0，则必然是窗户
  
  //采取无人机的前进方向为y方向，计算y方向，即uu值而言，跟无人机距离最近的障碍，并进行累计求和（x方向），若超过10则认为是一面墙
  //先从和无人机前方的uu开始积分
  for(i=uu_uav;i<globalMapWidth;i++)
  {

    for(size_t j=0;j<globalMapLongth;j++)
    { 
      //从y方向来看，正负不超过2即可   
      if(mazemap.maparry[i+j*globalMapWidth+ww_uav*globalMapLongth*globalMapLongth] == 1)
      {
        num_obstacle++;
      }
    }

    if(num_obstacle >10) //若是墙，则该行不少于10
    {
      ROS_INFO("Find the closest wall! The number is: %d", int(num_obstacle));
      break;
    }
    num_obstacle = 0;
  }
  uu_wall_start = i;
  //
  if(uu_wall_start == globalMapWidth) return;

  size_t uu_wall_min = uu_wall_start;
  for(size_t j=0; j<globalMapLongth; j++)
  {
      if(mazemap.maparry[(uu_wall_start-1)+j*globalMapWidth+ww_uav*globalMapLongth*globalMapLongth] == 1)
      {
        if(uu_wall_min <= uu_wall_start - 1) uu_wall_min = uu_wall_start-1;
      }
      if(mazemap.maparry[(uu_wall_start-2)+j*globalMapWidth+ww_uav*globalMapLongth*globalMapLongth] == 1)
      {
        if(uu_wall_min <= uu_wall_start - 2) uu_wall_min = uu_wall_start-2;
      }
  }

  for(size_t ww_temp = 0; ww_temp<globalMapHeight; ww_temp++)
  {
    for(size_t j=0; j<globalMapLongth; j++)
   {
      if(mazemap.maparry[uu_wall_start+j*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1 ||
      mazemap.maparry[(uu_wall_start+1)+j*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1 ||
      mazemap.maparry[(uu_wall_start+2)+j*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1 ||
      mazemap.maparry[(uu_wall_start-1)+j*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1 ||
      mazemap.maparry[(uu_wall_start-2)+j*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1)
      {
          obstacle_front_map.maparry[uu_wall_start+j*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] = 1;
          //ROS_INFO("Recording the wall!!!");
      }
   }
  }

  

  /*-----------------------------------------------寻找前面的墙的窗户-------------------------------------------------------*/
  //目前暂定的方法是从最下面的一排开始搜索，我认为这个方法可行，因为飞机一直保持1.5米的高度，只有三种情况
  //目前有一个过滤的方法，就是break_start_index所在的高度，保证在start点的前面或者是结束的位置之后至少有10个连续的障碍，这样才是墙
  size_t break_count;
  size_t break_start_index;
  size_t break_stop_index;
  size_t window_vv_value[globalMapHeight]={0};
  size_t window_vv_num[globalMapHeight]={0};
  size_t window_average_uu = 0;
  size_t window_average_vv = 0;
  for(size_t ww_temp=0; ww_temp<globalMapHeight; ww_temp++)
  {
    break_start_index = 0; 
    for(size_t vv_temp=0; vv_temp<globalMapLongth-10; vv_temp++) //vv_temp设置为了防止数组越界
    {
      size_t left_obstacle_count = 0;
      size_t right_obstacle_count = 0;

      if(obstacle_front_map.maparry[uu_wall_start+vv_temp*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1) //找到当前的障碍
      {
        if(obstacle_front_map.maparry[uu_wall_start+(vv_temp+1)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 0 &&      // 该障碍的右边为空,且长度为5以上，且右边第7~8个有障碍
        obstacle_front_map.maparry[uu_wall_start+(vv_temp+2)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 0 &&
        obstacle_front_map.maparry[uu_wall_start+(vv_temp+5)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 0 &&
        obstacle_front_map.maparry[uu_wall_start+(vv_temp+9)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1 &&
        obstacle_front_map.maparry[uu_wall_start+(vv_temp+10)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1 &&
        obstacle_front_map.maparry[uu_wall_start+(vv_temp+11)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1) 
        {
            break_start_index = vv_temp+1;
            if(obstacle_front_map.maparry[uu_wall_start+(vv_temp+5)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 0 &&
            obstacle_front_map.maparry[uu_wall_start+(vv_temp+6)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1) //假设窗户宽度为6
            {
              ROS_INFO("break 5");
              break_stop_index = vv_temp+5;
            }
            else if(obstacle_front_map.maparry[uu_wall_start+(vv_temp+6)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 0 &&
            obstacle_front_map.maparry[uu_wall_start+(vv_temp+7)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1) //假设窗户宽度为6
            {
              break_stop_index = vv_temp+6;
              ROS_INFO("break 6");
            }
            else if(obstacle_front_map.maparry[uu_wall_start+(vv_temp+7)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 0 &&
            obstacle_front_map.maparry[uu_wall_start+(vv_temp+8)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1) //假设窗户宽度为7
            {
              break_stop_index = vv_temp+7;
              ROS_INFO("break 7");
            }
            else if(obstacle_front_map.maparry[uu_wall_start+(vv_temp+8)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 0 &&
            obstacle_front_map.maparry[uu_wall_start+(vv_temp+9)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1) //假设窗户宽度为7
            {
              break_stop_index = vv_temp+8;
              ROS_INFO("break 8");
            }
            for(size_t temp_left=1; break_start_index-temp_left>=0; temp_left++)
            {
              if(obstacle_front_map.maparry[uu_wall_start+(break_start_index-temp_left)*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1)
                left_obstacle_count++;
                else
                break;

            }
            if(break_stop_index>0)
              for(size_t temp_right=break_stop_index+1; temp_right<=globalMapLongth; temp_right++)
              {
                if(obstacle_front_map.maparry[uu_wall_start+temp_right*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] == 1)
                  right_obstacle_count++;
                  else
                  break;

              }
              ROS_INFO("right_obstacle_count %d",int(right_obstacle_count));
            if(left_obstacle_count>=10||right_obstacle_count>=10)
            {            
              break_start_index = vv_temp+1;
              ROS_INFO("Find window!");
            }
            else
            break_start_index = 0;
        }

          

      }


      //计算窗口的中心位置

      if(break_start_index!=0)
      {
        if(vv_temp >= break_start_index && vv_temp <= break_start_index+6)
          window_front_map.maparry[uu_wall_start+vv_temp*globalMapWidth+ww_temp*globalMapLongth*globalMapLongth] = 1;
          //统计红色横的中点的位置,位置id在window_vv_value[]中,每个id的个数在对应的window_vv_num
          size_t temp_window_vv_value=(break_start_index+break_stop_index)/2;
          size_t temp_v=0;
          for(temp_v;window_vv_value[temp_v]!=0;temp_v++)
          {
            if(temp_window_vv_value==window_vv_value[temp_v])
            {
              window_vv_num[temp_v]++;
              break;
            }
          }
          if(window_vv_value[temp_v]==0)
          {
            window_vv_value[temp_v]=temp_window_vv_value;
            window_vv_num[temp_v]=1;
          } 

      }


    }//扫描完一行
   
      
  }//扫描完一个墙

  //去window_vv_num最大的作为窗位置
  size_t max_id=0;
  size_t max_value=0;
  for(size_t i=0;i<16;i++)
  {
    if(window_vv_num[i]>max_value)
    {
      max_value=window_vv_num[i];
      max_id=i;
    }
      
  }
  if(window_vv_num[max_id]>2)
  {
     window_average_vv=window_vv_value[max_id];
    ROS_WARN("window_average_vv %d",int(window_average_vv));
  }

  





  /*
  bool break_detect = 0; //检测到隔断
  bool left_wall_detect = 0; //检测到左侧的墙
  bool right_wall_detect = 0; //检测到右侧的墙
  for(size_t j=0;j<100;j++)
  {
    if(mazemap.maparry[i+j*globalMapWidth+ww_uav*globalMapLongth*globalMapLongth] == 1 &&  
    right_wall_detect == 0 &&
    break_detect == 0)
    {
      obstacle_front_map_left.maparry[i+j*globalMapWidth+ww_uav*globalMapLongth*globalMapLongth]=1;
      left_wall_detect = 1;
    }
    if(right_wall_detect == 0) //防止数组越界
    {    
      if(mazemap.maparry[i+j*globalMapWidth+ww_uav*globalMapLongth*globalMapLongth] == 0 &&
      mazemap.maparry[i+(j+1)*globalMapWidth+ww_uav*globalMapLongth*globalMapLongth] == 0 &&
      left_wall_detect == 1)
      {
        break_detect = 1; //检测到了间断
      }
    }
    if(mazemap.maparry[i+j*globalMapWidth+ww_uav*globalMapLongth*globalMapLongth] == 1 && 
    right_wall_detect == 0 &&
    break_detect == 1)
    {
      obstacle_front_map_right.maparry[i+j*globalMapWidth+ww_uav*globalMapLongth*globalMapLongth]=1;
      right_wall_detect = 1;
    }
  }
  */


  //得到窗口的范围，计算窗的中心位置






  









  /*------------------------------------发布栅格地图----------------------------------------------*/
  //发布和无人机同一高度的前方栅格地图
  //obstacle_front_left_pub.publish(obstacle_front_map_left);
  //obstacle_front_right_pub.publish(obstacle_front_map_right);
  obstacle_front_pub.publish(obstacle_front_map);
  //发布探测到的窗口的栅格地图
  window_front_pub.publish(window_front_map);
}

// Subscribe the odometry location of UAV
void build_3D_map::uav_loc_cb(const nav_msgs::Odometry::ConstPtr& uav_locat_msg)
{
  uav_location.point.x  = uav_locat_msg->pose.pose.position.x;//position.x;
  uav_location.point.y  = uav_locat_msg->pose.pose.position.y;//position.y;
  uav_location.point.z  = uav_locat_msg->pose.pose.position.z;//position.z;
  ROS_INFO("uav_location_x: %lf uav_location_y: %lf uav_location_z: %lf", uav_location.point.x,uav_location.point.y,uav_location.point.z);
}

int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "build_3D_map");
  for(size_t i=0;i<160000;i++)
  {  
    mazemap.maparry[i]=0;
    uav_map.maparry[i]=0;
    //obstacle_front_map_left.maparry[i]=0;
    //obstacle_front_map_right.maparry[i]=0;
    obstacle_front_map.maparry[i]=0;
  }


  build_3D_map ic;
	
	//ros::spin();
	return 0;
}

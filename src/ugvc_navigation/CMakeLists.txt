cmake_minimum_required(VERSION 2.8.3)
project(ugvc_navigation)
find_package(catkin REQUIRED COMPONENTS
  message_generation
	roscpp
	rospy
	std_msgs
	gazebo_ros
	move_base
	map_server
	amcl
	gmapping
	dwa_local_planner
  pcl_ros
  pcl_conversions
  
 # sbg617d
)
add_message_files(FILES
  flight_command.msg
  front_obstacle.msg
  mymap.msg
)
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS
  message_runtime 
  roscpp
  std_msgs
)

#以下语句非常重要，用于引用其他包共享的头文件
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include 
	${catkin_INCLUDE_DIRS}
)
# add_dependencies(easy_build_map 
# 	${${PROJECT_NAME}_EXPORTED_TARGETS} 
# 	${catkin_EXPORTED_TARGETS}
# )

add_executable(tf_laser
  src/tf_laser.cpp
)
add_dependencies(tf_laser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_laser
  ${catkin_LIBRARIES}
)
#add_dependencies(easy_build_map2 )






add_executable(build_3D_map_pub
  include/ugvc_navigation/build_3D_map.h
  src/build_3D_map_pub.cpp
)
add_dependencies(build_3D_map_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(build_3D_map_pub
  ${catkin_LIBRARIES}
)

add_executable(test_odom
  src/test_odom.cpp
)
add_dependencies(test_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_odom
  ${catkin_LIBRARIES}
)

add_executable(convert2image
  src/convert2image.cpp
)
add_dependencies(convert2image ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(convert2image
  ${catkin_LIBRARIES}
)


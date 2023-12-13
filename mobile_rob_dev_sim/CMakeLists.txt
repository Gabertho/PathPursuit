cmake_minimum_required(VERSION 2.8.3)
project(mobile_rob_dev_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  ackermann_msgs
  tf2
  tf2_ros
  rospy
)

find_package(orocos_kdl REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs ackermann_msgs tf2 tf2_ros rospy
  DEPENDS OpenCV orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar_node src/astar_node.cpp)
target_link_libraries(astar_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(pure_pursuit src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


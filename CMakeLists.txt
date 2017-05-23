cmake_minimum_required(VERSION 2.8.3)
project(slammin)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package( OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
	pcl_ros
	roscpp
	rospy
	std_msgs
	tf
	cv_bridge
	image_transport
	message_generation
)

add_message_files(
   FILES
   point3d.msg
   pointVector3d.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  pcl_ros
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slammin
#  CATKIN_DEPENDS pcl_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


 add_executable(${PROJECT_NAME}_node src/kati.cpp)
target_link_libraries(${PROJECT_NAME}_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloudlistener_toMap src/cloudlistener_toMap.cpp)
target_link_libraries(cloudlistener_toMap ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${PCL_LIBRARIES})

add_executable(slammin_matcher src/slammin_matcher.cpp)
target_link_libraries(slammin_matcher ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${PCL_LIBRARIES})


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
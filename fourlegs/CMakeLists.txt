cmake_minimum_required(VERSION 2.8.3)
project(fourlegs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fourlegs
  CATKIN_DEPENDS 
pcl_conversions 
pcl_ros 
roscpp 
sensor_msgs 
std_msgs
tf
tf_conversions
eigen_conversions
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
${Eigen_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)


add_executable(zslam src/zslam/main.cpp 
					src/zslam/pcl_reciver.cpp 
					src/zslam/tf_reciver.cpp
					include/zslam.h)
target_link_libraries(zslam ${catkin_LIBRARIES})
add_dependencies(zslam ${catkin_EXPORTED_TARGETS})

add_executable(plane_detector src/plane_detector/main.cpp
					src/plane_detector/pcl_reciver.cpp
					src/plane_detector/get_plane_model.cpp
					src/plane_detector/get_plane_angle.cpp
					src/plane_detector/average.cpp
					src/plane_detector/step_modeling_horizontal.cpp
					src/plane_detector/feet_pcl_reciver.cpp
					include/plane_detector.h)
target_link_libraries(plane_detector ${catkin_LIBRARIES})
add_dependencies(plane_detector ${catkin_EXPORTED_TARGETS})


add_executable(robotodom2 src/robotodom2/main.cpp
					src/robotodom2/pcl_reciver.cpp
					include/robotodom2.h)
target_link_libraries(robotodom2 ${catkin_LIBRARIES})
add_dependencies(robotodom2 ${catkin_EXPORTED_TARGETS})



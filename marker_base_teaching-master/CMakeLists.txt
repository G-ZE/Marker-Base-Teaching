cmake_minimum_required(VERSION 2.8.3)
project(marker_base_teaching)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  tf_conversions
  image_transport
  message_generation
  geometry_msgs
  sensor_msgs
)

find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system thread)
find_package(PCL 1.7.0 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  markers.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  pcl_ros
  tf_conversions
  image_transport
  CATKIN_DEPENDS message_runtime
)




include_directories(${PCL_INCLUDE_DIRS}) 
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OPENGL_INCLUDE_DIR}  ${GLUT_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})




#add_executable(kinect_retrieve src/kinect_retrieve.cpp)
#target_link_libraries(kinect_retrieve yaml-cpp ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(kinect_retrieve ${catkin_EXPORTED_TARGETS})

#add_executable(single_pose_estimation src/single_pose_estimation.cpp)
#target_link_libraries(single_pose_estimation yaml-cpp ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(single_pose_estimation ${catkin_EXPORTED_TARGETS})

add_executable(testing src/testing.cpp)
target_link_libraries(testing yaml-cpp ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(testing ${catkin_EXPORTED_TARGETS})

add_executable(kinect_pose_estimation src/kinect_pose_estimation.cpp)
target_link_libraries(kinect_pose_estimation yaml-cpp ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(kinect_pose_estimation ${catkin_EXPORTED_TARGETS})

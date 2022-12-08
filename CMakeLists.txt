cmake_minimum_required(VERSION 3.0.2)
project(pointcloud)
#set(CMAKE_CXX_FLAGS -std=c++14)

SET (BOOST_ROOT "/usr/include/boost")
SET (BOOST_INCLUDEDIR "/usr/include/boost")
#SET (LIBPOINTMATCHER_ROS_DIR "/usr/local/share/libpointmatcher/cmake") #/libpointmatcher-config.cmake
#SET (BOOST_LIBRARYDIR "/usr/local/boost/lib")



find_package(catkin REQUIRED COMPONENTS
  roscpp #
  rospy #
  roslib
  sensor_msgs
  dynamic_reconfigure
  geometry_msgs
  stereo_msgs
  std_msgs #
  pcl_ros #
  cv_bridge
  image_transport
  camera_info_manager
  #irob_msgs
  #irob_utils
  libpointmatcher
)

find_package(Eigen3 REQUIRED)
#find_package(tf2 REQUIRED)
#find_package(tf2_geometry_msgs REQUIRED)
#find_package(tf2_ros REQUIRED)
#find_package(pcl_conversions REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(rclcpp_components REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)

find_package(FLANN REQUIRED) #
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(PCL 1.2 REQUIRED)
find_package(Matplot++ REQUIRED)

find_package(sciplot)

find_package(realsense2 REQUIRED)
find_package(libpointmatcher 1.1.0 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
 #
include_directories(SYSTEM
  ${HDF5_INCLUDE_DIR}
) #
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("${libpointmatcher_INCLUDE_DIRS}")


#set(dependencies
#  pcl_conversions
#  rclcpp
#  sensor_msgs
#  geometry_msgs
#  tf2
#  tf2_geometry_msgs
#  tf2_ros
#  EIGEN3
#  PCL
#)

#add_library(pcl_ros_tf src/transforms.cpp)
#target_include_directories(pcl_ros_tf PUBLIC
 #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 #   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
#)
#ament_target_dependencies(pcl_ros_tf
#  ${dependencies}
#)




add_executable(NearestNeighbour src/NearestNeighbour.cpp)
add_executable(normal_estimation src/normal_estimation.cpp)
add_executable(voxel_grid src/voxel_grid.cpp)
add_executable(P_Matching src/P_Matching.cpp)
add_executable(P_GeomCons src/P_GeomCons.cpp)
add_executable(RemoveNAN src/RemoveNAN.cpp)
add_executable(passthrough src/Filter_Passthrough.cpp)
add_executable(outlier_removal src/Outlier_removal.cpp)
add_executable(upsampling src/upsampling.cpp)
add_executable(downsampling src/downsampling.cpp)
add_executable(Surface_smoothing src/Surface_smoothing.cpp)
add_executable(euclidean_segment src/euclidean_segment.cpp)
add_executable(colorbased_segment src/colorbased_segment.cpp)



target_link_libraries(NearestNeighbour ${PCL_LIBRARIES})
target_link_libraries(normal_estimation ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(voxel_grid ${PCL_LIBRARIES})
target_link_libraries(P_Matching ${PCL_LIBRARIES})
target_link_libraries(P_GeomCons ${PCL_LIBRARIES})
target_link_libraries(RemoveNAN ${PCL_LIBRARIES})
target_link_libraries(passthrough ${PCL_LIBRARIES})
target_link_libraries(outlier_removal ${PCL_LIBRARIES})
target_link_libraries(upsampling ${PCL_LIBRARIES})
target_link_libraries(downsampling ${PCL_LIBRARIES})
target_link_libraries(Surface_smoothing ${PCL_LIBRARIES})
target_link_libraries(euclidean_segment ${PCL_LIBRARIES})
target_link_libraries(colorbased_segment ${PCL_LIBRARIES})



add_executable(P_SHOT src/P_SHOT.cpp)
target_link_libraries(P_SHOT ${PCL_LIBRARIES})

add_executable(P_PFH src/P_PFH.cpp)
target_link_libraries(P_PFH ${PCL_LIBRARIES})

add_executable(P_FPFH src/P_FPFH.cpp)
target_link_libraries(P_FPFH ${PCL_LIBRARIES})


add_executable(P_ESF src/P_ESF.cpp)
target_link_libraries(P_ESF ${PCL_LIBRARIES})

add_executable(P_VFH src/P_VFH.cpp)
target_link_libraries(P_VFH ${PCL_LIBRARIES})

add_executable(P_CVFH src/P_CVFH.cpp)
target_link_libraries(P_CVFH ${PCL_LIBRARIES})

add_executable(correspondence_grouping src/correspondence_grouping.cpp)
target_link_libraries(correspondence_grouping ${PCL_LIBRARIES})

add_executable(keypoint src/Keypoint.cpp)
target_link_libraries(keypoint ${PCL_LIBRARIES})

add_executable (resampling src/resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})

add_executable(depth_pcl src/depth_pcl.cpp)
target_link_libraries(depth_pcl ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (planesegment src/P_planesegment.cpp)
target_link_libraries (planesegment ${PCL_LIBRARIES})


add_executable(localpipeline src/P_localpipeline.cpp)
target_link_libraries(localpipeline ${PCL_LIBRARIES})

add_executable(localpipelineSHOT src/P_localpipelineSHOT.cpp)
target_link_libraries(localpipelineSHOT ${PCL_LIBRARIES})

add_executable(localpipelinePFH src/P_localpipelinePFH.cpp)
target_link_libraries(localpipelinePFH ${PCL_LIBRARIES})

add_executable(localpipelineFPFH src/P_localpipelineFPFH.cpp)
target_link_libraries(localpipelineFPFH ${PCL_LIBRARIES})

add_executable (pcd_quality src/PCD_quality.cpp)
target_link_libraries(pcd_quality ${PCL_LIBRARIES})

add_executable (colorbased_regiongrowing src/colorbased_regiongrowing.cpp)
target_link_libraries(colorbased_regiongrowing ${PCL_LIBRARIES})

add_executable (centroid_distance src/centroid_distance.cpp)
target_link_libraries(centroid_distance ${PCL_LIBRARIES} Matplot++::matplot)

add_executable (boxplot src/boxplot.cpp)
target_link_libraries(boxplot ${PCL_LIBRARIES} Matplot++::matplot)

find_package(HDF5 REQUIRED)
if(HDF5_FOUND)

  find_package(FLANN)
  include_directories(${FLANN_INCLUDE_DIRS})

  include_directories(${HDF5_INCLUDE_DIR})

  add_executable(globalpipelineVFH1 src/P_globalpipelineVFH.cpp)
  target_link_libraries(globalpipelineVFH1 ${PCL_LIBRARIES} ${Boost_LIBRARIES}
                                 ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})

endif(HDF5_FOUND)


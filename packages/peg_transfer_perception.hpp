#ifndef PEG_TRANSFER_PERCEPTION_H
#define PEG_TRANSFER_PERCEPTION_H


#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>

//#include "irob_utils/tool_pose.hpp"
//#include "irob_utils/utils.hpp"
//#include "irob_utils/abstract_directions.hpp"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_grabber.h>

#include "pointmatcher/PointMatcher.h"

#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"
#include "pointmatcher/get_params_from_server.h"
#include "pointmatcher/ros_logger.h"

namespace saf {

class PegTransferPerception
{
private:
  ros::NodeHandle nh;
  ros::Publisher pcl_pub;
  ros::Publisher obj_pub;
  std::string ply_filename;
  std::string configfile;
  int hue_lower;
  int hue_upper;
  int saturation_lower;
  int saturation_upper;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);


public:
  PegTransferPerception(ros::NodeHandle, std::string, std::string, int, int, int, int);
  void runPerception();

};

}

#endif // PEG_TRANSFER_PERCEPTION_H


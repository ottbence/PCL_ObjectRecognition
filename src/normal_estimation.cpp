#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <ros/time.h>


int
main(int argc, char** argv)
{
	ros::init(argc, argv, "normal_estimation");
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ros::NodeHandle nh;
	//ros::Duration d(1.0);
	//ros::Rate loop_rate(10);
	//ros::Time::init();

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Object for normal estimation.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	// For every point, use all neighbors in a radius of 3cm.
	normalEstimation.setRadiusSearch(0.03);
	// A kd-tree is a data structure that makes searches efficient. More about it later.
	// The normal estimation object will use it to find nearest neighbors.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);

	// Calculate the normals.
	normalEstimation.compute(*normals);

	// Visualize them.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	// Display one normal out of 20, as a line of length 3cm.
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 20, 0.03, "normals");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		ros::Duration(1).sleep();
		//loop_rate.sleep();
		//std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}


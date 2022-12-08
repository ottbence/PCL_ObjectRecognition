#include <iostream>
#include <thread>
#include <vector>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>



using namespace std::chrono_literals;

int
main (int argc, char** argv)
{
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PCDWriter writer;
  // Read a PCD file from disk.
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0)
  {
		return -1;
  }
  pcl::copyPointCloud(*cloud, *cloud_filtered);
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud (*cloud, *indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (600);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    pcl::io::savePCDFileASCII(ss.str (), *cloud_cluster);
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
  
  

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  //viewer.showCloud (colored_cloud);
  
  pcl::io::savePCDFileASCII(argv[2], *colored_cloud);
  pcl::visualization::PCLVisualizer viewer1 ("Color based clusters");
  viewer1.addPointCloud (colored_cloud, "colored clusters");
  while (!viewer1.wasStopped ())
  {
    viewer1.spinOnce ();
  }

  return (0);
}

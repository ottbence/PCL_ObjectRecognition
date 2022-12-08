#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Filter object.
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	// We set the size of every voxel to be 1x1x1cm
	// (only one point per every cubic centimeter will survive).
	filter.setLeafSize(0.1f, 0.1f, 0.1f);

	filter.filter(*filteredCloud);
}

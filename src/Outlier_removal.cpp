#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	// Set number of neighbors to consider to 50.
	filter.setMeanK(50);
	// Set standard deviation multiplier to 1.
	// Points with a distance larger than 1 standard deviation of the mean distance will be outliers.
	filter.setStddevMulThresh(1.0);

	filter.filter(*filteredCloud);
	pcl::io::savePCDFileASCII(argv[2], *filteredCloud);
}

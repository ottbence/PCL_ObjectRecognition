#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
typedef pcl::PointXYZRGB PointType;
int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr plane(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr convexHull(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr objects(new pcl::PointCloud<PointType>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<PointType>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Get the plane model, if present.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<PointType> segmentation;
	segmentation.setInputCloud(cloud);
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.01);
	segmentation.setOptimizeCoefficients(true);
	pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
	segmentation.segment(*planeIndices, *coefficients);

	if (planeIndices->indices.size() == 0)
		std::cout << "Could not find a plane in the scene." << std::endl;
	else
	{
		// Copy the points of the plane to a new cloud.
		pcl::ExtractIndices<PointType> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(planeIndices);
		extract.setNegative (false); // FALSE!!
		extract.filter(*plane);

		// Retrieve the convex hull.
		pcl::ConvexHull<PointType> hull;
		hull.setInputCloud(cloud); // plane
		// Make sure that the resulting hull is bidimensional.
		hull.setDimension(2);
		hull.reconstruct(*convexHull);

		// Redundant check.
		if (hull.getDimension() == 2)
		{
			// Prism object.
			pcl::ExtractPolygonalPrismData<PointType> prism;
			prism.setInputCloud(cloud);
			prism.setInputPlanarHull(convexHull);
			// First parameter: minimum Z value. Set to 1cm, segments objects lying on the plane (can be negative).
			// .1: 1.0; .2: 1.15f ;.4: 1.1f ; .6: 0.5f; .7: 0.9; .8: 1.15; .12: 1.0; 
			// Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect. 
			// .2: 1.3f; .4: 2.0f; .6: 2.0 ; .7: 2
			prism.setHeightLimits(0.1f, 1.6f);
			pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

			prism.segment(*objectIndices);

			// Get and show all points retrieved by the hull.
			extract.setIndices(objectIndices);
			extract.filter(*objects);
			pcl::visualization::CloudViewer viewerObjects("Objects on table");
			viewerObjects.showCloud(objects);
			// Save it back.
			pcl::io::savePCDFileASCII(argv[2], *objects);
			while (!viewerObjects.wasStopped())
			{
				// Do nothing but wait.
			}
		}
		else std::cout << "The chosen hull is not planar." << std::endl;
	}
}

#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <iostream>

int
main(int argc, char** argv)
{
	// Objects for storing the keypoints of the scene and the model.
	pcl::PointCloud<pcl::PointXYZ>::Ptr sceneKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr modelKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	// Objects for storing the unclustered and clustered correspondences.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
	std::vector<pcl::Correspondences> clusteredCorrespondences;
	// Object for storing the transformations (rotation plus translation).
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;

	// Read the keypoints from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *sceneKeypoints) != 0)
	{
		return -1;
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *modelKeypoints) != 0)
	{
		return -1;
	}

	// Note: here you would compute the correspondences.
	// It has been omitted here for simplicity.

	// Object for correspondence grouping.
	pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> grouping;
	grouping.setSceneCloud(sceneKeypoints);
	grouping.setInputCloud(modelKeypoints);
	grouping.setModelSceneCorrespondences(correspondences);
	// Minimum cluster size. Default is 3 (as at least 3 correspondences
	// are needed to compute the 6 DoF pose).
	grouping.setGCThreshold(3);
	// Resolution of the consensus set used to cluster correspondences together,
	// in metric units. Default is 1.0.
	grouping.setGCSize(0.01);

	grouping.recognize(transformations, clusteredCorrespondences);

	std::cout << "Model instances found: " << transformations.size() << std::endl << std::endl;
	for (size_t i = 0; i < transformations.size(); i++)
	{
		std::cout << "Instance " << (i + 1) << ":" << std::endl;
		std::cout << "\tHas " << clusteredCorrespondences[i].size() << " correspondences." << std::endl << std::endl;

		Eigen::Matrix3f rotation = transformations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = transformations[i].block<3, 1>(0, 3);
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		std::cout << std::endl;
		printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}
}

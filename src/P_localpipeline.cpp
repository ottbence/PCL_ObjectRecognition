#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <time.h>
#include <iostream>
#include <pcl/keypoints/iss_3d.h>
#include <cstdlib>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/registration/sample_consensus_prerejective.h> 
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/pcl_macros.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/common/centroid.h>
#include <fstream>

// sift
namespace pcl
{
    template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
        operator () (const PointXYZ &p) const
        {
            return p.z;
        }
    };
}


typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_;
std::string scene_filename_;
std::string descriptortype;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);
// Descriptor types
bool shot = true;
bool fpfh = false;
bool pfh = false;
bool pfhrgb = false;
// Normal estimation
bool ShotOMP = true;
bool normal_3d = false;
// Keypoint extraction methods
bool uniform = true;
bool iss = false;
bool harris = false;
bool sift_k = false;
bool voxel = false;
//
// COMPUTING CLOUD RESOLUTION
//
double
computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  //std::cout << "Computed resolution :" << res << std::endl;
  return res;
}

///
/// COMPUTING DESCRIPTORS
///
void
compute_shot(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::SHOT352>::Ptr descriptors, std::string src)
{
    clock_t start=clock();

    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setNumberOfThreads(4);  
    descr_est.setRadiusSearch (descr_rad_);   

    descr_est.setInputCloud (keypoints); 
    descr_est.setInputNormals (normals);  
    descr_est.setSearchSurface (cloud);       
    descr_est.compute(*descriptors);   

    clock_t end=clock();
    cout<<"Time SHOT: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"Get SHOT  " << src << ": " <<descriptors->points.size()<<endl;
}

void
compute_pfh(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
             pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors, std::string src)
{
    clock_t start = clock();

    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ> );
    pfh.setInputCloud(keypoints);  
    pfh.setInputNormals(normals);  
    pfh.setSearchSurface(cloud); 
    pfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    pfh.setRadiusSearch(0.5);	// 0.5 OR descr_rad_
    pfh.compute(*descriptors);
    clock_t end = clock();
    
    cout<<"Time pfh: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"Get pfh "<< src << ": "<<descriptors->points.size()<<endl;

}

void
compute_fpfh(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors, std::string src)
{
    clock_t start = clock();
    // FPFH estimation object.
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ> );
    fpfh.setInputCloud(keypoints); 
    fpfh.setInputNormals(normals);   
    fpfh.setSearchSurface(cloud); 
    fpfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    fpfh.setKSearch(10);    // 10 OR descr_rad_
    fpfh.compute(*descriptors);
    clock_t end = clock();
    cout<<"Time fpfh: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"Get fpfh "<< src << ": "<<descriptors->points.size()<<endl;
}


///
/// MATCHING DESCRIPTORS
///
void
find_match_shot(pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors,pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors,
        pcl::CorrespondencesPtr model_scene_corrs)
{
    clock_t start = clock();
    pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    match_search.setInputCloud(model_descriptors);

    for (size_t i = 0; i < scene_descriptors->size(); ++i)
    {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        // Ignore NaNs.
        if (std::isfinite(scene_descriptors->at(i).descriptor[0] ))
        {

            // Find the nearest neighbor (in descriptor space)...
            int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
            // ...and add a new correspondence if the distance is less than a threshold
            // (SHOT distances are between 0 and 1, other descriptors use different metrics).
            if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
            {
                pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
                model_scene_corrs->push_back(corr);
            }
        }
    }

    
    if(model_scene_corrs->size() ==0)
        pcl::console::print_warn ("\nNo SHOT correspondences !!\n");
    else
        std::cout << "Found " << model_scene_corrs->size() << " SHOT correspondences." << std::endl;
    clock_t end = clock();
    cout<<"Time match: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"-----------------------------"<<endl;
        
}

void
find_match_pfh(pcl::PointCloud<pcl::PFHSignature125>::Ptr model_descriptors,pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_descriptors,
        pcl::CorrespondencesPtr model_scene_corrs)
{
    clock_t start = clock();
    pcl::KdTreeFLANN<pcl::PFHSignature125> match_search;
    match_search.setInputCloud(model_descriptors);

    for (size_t i = 0; i < scene_descriptors->size(); ++i)
    {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        // Ignore NaNs.
        if (std::isfinite(scene_descriptors->at(i).histogram[0] ))
        {
            // Find the nearest neighbor (in descriptor space)...
            int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
            // ...and add a new correspondence if the distance is less than a threshold
            // (SHOT distances are between 0 and 1, other descriptors use different metrics).
            if (found_neighs == 1 && neigh_sqr_dists[0] < 0.1f)
            {
                pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
                model_scene_corrs->push_back(corr);
            }
        }
    }

    
    if(model_scene_corrs->size() ==0)
        pcl::console::print_warn ("\nNo PFH correspondences !!\n");
    else
        std::cout << "Found " << model_scene_corrs->size() << " PFH correspondences." << std::endl;
    clock_t end = clock();
    cout<<"Time match: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"-----------------------------"<<endl;
        
}


void
find_match_fpfh(pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_descriptors,pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_descriptors,
        pcl::CorrespondencesPtr model_scene_corrs)
{
    clock_t start = clock();
    pcl::KdTreeFLANN<pcl::FPFHSignature33> match_search;
    match_search.setInputCloud(model_descriptors);

    for (size_t i = 0; i < scene_descriptors->size(); ++i)
    {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        // Ignore NaNs.
        if (std::isfinite(scene_descriptors->at(i).histogram[0] ))
        {
            // Find the nearest neighbor (in descriptor space)...
            int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
            // ...and add a new correspondence if the distance is less than a threshold
            // (SHOT distances are between 0 and 1, other descriptors use different metrics).
            if (found_neighs == 1 && neigh_sqr_dists[0] < 0.1f)
            {
                pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
                model_scene_corrs->push_back(corr);
            }
        }
    }

    
    if(model_scene_corrs->size() ==0)
        pcl::console::print_warn ("\nNo FPFH correspondences !!\n");
    else
        std::cout << "Found " << model_scene_corrs->size() << " FPFH correspondences." << std::endl;
    clock_t end = clock();
    cout<<"Time match: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"-----------------------------"<<endl;
        
}


///
/// KEYPOINT EXTRACTION METHODS
///

void uniform_sampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_result, bool Model)
{
  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (cloud);
  if (Model)
  {
    uniform_sampling.setRadiusSearch (model_ss_);
  }
  else 
  {
    uniform_sampling.setRadiusSearch (scene_ss_);
  }
  uniform_sampling.filter (*keypoints_result);
  
  
  if (Model)
  {
    std::cout << "Model total points: " << cloud->size () << "; Selected Keypoints: " << keypoints_result->size () << std::endl;
  }
  else 
  {
    std::cout << "Scene total points: " << cloud->size () << "; Selected Keypoints: " << keypoints_result->size () << std::endl;
  }
}

void
compute_iss(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, std::string src)
{
    clock_t start = clock();
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_det;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    double model_resolution = computeCloudResolution(cloud);
    cout<<"resolution: "<<model_resolution<<endl;

    iss_det.setMinNeighbors(10);
    iss_det.setThreshold21(0.975);
    iss_det.setThreshold32(0.975);
    iss_det.setNumberOfThreads(4);

    iss_det.setInputCloud(cloud);
    iss_det.setSearchMethod(tree);
    iss_det.setSalientRadius(6*model_resolution);  // 0.5
    iss_det.setNonMaxRadius(4*model_resolution);
    iss_det.compute(*keypoints);


    clock_t end = clock();
    cout << "iss time：" << (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout << "iss keypoint size "<< src << ": " << keypoints->size() << endl;

}

void compute_harris(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_result, std::string src)
{
    pcl::HarrisKeypoint3D <pcl::PointXYZ, pcl::PointXYZI> detector;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
    detector.setNonMaxSupression (true);
    detector.setInputCloud (cloud);
    detector.setRadius(0.01f);	//0.1f
    detector.setRadiusSearch(0.01f); //0.1f
    pcl::StopWatch watch;
    detector.compute (*keypoints);
    pcl::console::print_highlight ("Detected %zd points in %lfs\n", keypoints->size (), watch.getTimeSeconds ());
    pcl::PointIndicesConstPtr keypoints_indices = detector.getKeypointsIndices ();
    if (!keypoints_indices->indices.empty ())
    { 
        pcl::copyPointCloud(*cloud,keypoints_indices->indices,*keypoints_result);  
    }
    else
        pcl::console::print_warn ("Keypoints indices are empty!\n");
}

void
compute_sift(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, std::string src)
{
    clock_t start = clock();
    const float min_scale = 0.01f;
    const int n_octaves = 3;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.001f;

    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    sift.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.compute(result);
    
    pcl::copyPointCloud(result, *keypoints);
    clock_t end = clock();

    cout << "sift time：" << (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout << "sift keypoint size"  << src << ": "<< keypoints->size() << endl;
}

void voxel_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filteredcloud)
{
   pcl::VoxelGrid<pcl::PointXYZ> filter;
   filter.setInputCloud(cloud);
	// We set the size of every voxel to be 1x1x1cm
	// (only one point per every cubic centimeter will survive).
   filter.setLeafSize(0.01f, 0.01f, 0.01f);
   filter.filter(*filteredcloud);
   cout << "Downsampled cloud size : " << filteredcloud->size() << endl;
}


void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show used keypoints." << std::endl;
  std::cout << "     -c:                     Show used correspondences." << std::endl;
  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "     --descriptors (shot|fpfh|pfh)          Descriptor type (default shot)." << std::endl;
  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --keypoint (uniform|iss|harris|sift): Keypoint extraction method (default Uniform sampling)." << std::endl;
  std::cout << "     --normal_type (ShotOMP|normal_3d)     Cloud normal estimation type (default ShotOMP)" << std::endl;
  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
  
}

void
parseCommandLine (int argc, char *argv[])
{
  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 2)
  {
    std::cout << "Filenames missing.\n";
    showHelp (argv[0]);
    exit (-1);
  }

  model_filename_ = argv[filenames[0]];
  scene_filename_ = argv[filenames[1]];

  //Program behavior
  if (pcl::console::find_switch (argc, argv, "-k"))
  {
    show_keypoints_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-c"))
  {
    show_correspondences_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-r"))
  {
    use_cloud_resolution_ = true;
  }

  std::string used_algorithm;
  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
  {
    if (used_algorithm.compare ("Hough") == 0)
    {
      use_hough_ = true;
    }else if (used_algorithm.compare ("GC") == 0)
    {
      use_hough_ = false;
    }
    else
    {
      std::cout << "Wrong algorithm name.\n";
      showHelp (argv[0]);
      exit (-1);
    }
  }
  
  std::string used_descriptors;
  if (pcl::console::parse_argument (argc, argv, "--descriptors", used_descriptors) != -1)
  {
    if (used_descriptors.compare ("shot") == 0)
    {
      shot = true;
    }
    else if (used_descriptors.compare ("fpfh") == 0)
    {
      shot = false;
      fpfh = true;
    }
    else if (used_descriptors.compare ("pfh") == 0)
    {
      shot = false;
      pfh = true;
    }
    else if (used_descriptors.compare ("pfhrgb") == 0)
    {
      shot = false;
      pfhrgb = true;
    }
    else
    {
      std::cout << "Wrong descriptors type .\n";
      showHelp (argv[0]);
      exit (-1);
    }
  }
  
  std::string used_normal;
  if (pcl::console::parse_argument (argc, argv, "--normal_type", used_normal) != -1)
  {
    if (used_normal.compare ("ShotOMP") == 0)
    {
      ShotOMP = true;
    }
    else if (used_normal.compare ("normal_3d") == 0)
    {
      ShotOMP = false;
      normal_3d = true;
    }
    else
    {
      std::cout << "Wrong normal type .\n";
      showHelp (argv[0]);
      exit (-1);
    }
  }
  
  std::string used_keypoint;
  if (pcl::console::parse_argument (argc, argv, "--keypoint", used_keypoint) != -1)
  {
    if (used_keypoint.compare ("uniform") == 0)
    {
      uniform = true;

    }
    else if (used_keypoint.compare ("iss") == 0)
    {
      uniform = false;
      voxel = true;
      iss = true;
    }
    else if (used_keypoint.compare ("harris") == 0)
    {
      uniform = false;
      voxel = true;
      harris = true;
    }
    else if (used_keypoint.compare ("sift") == 0)
    {
      uniform = false;
      voxel = true;
      sift_k = true;
    }
    else
    {
      std::cout << "Wrong keypoint type .\n";
      showHelp (argv[0]);
      exit (-1);
    }
  }

  //General parameters
  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
}



int
main (int argc, char *argv[])
{
  parseCommandLine (argc, argv);
  
  // Model and scene pointclouds
  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  
  // Filtered cloud for voxel grid downsampling
  pcl::PointCloud<PointType>::Ptr filtered_cloud_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr filtered_cloud_scene (new pcl::PointCloud<PointType> ());
  
  // Model-scene correspondences
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
  
  // Keypoints
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  
  // Normals
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  
  // SHOT descriptors
  pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors_SHOT (new pcl::PointCloud<pcl::SHOT352> ());
  pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors_SHOT (new pcl::PointCloud<pcl::SHOT352> ());
  // FPFH descriptors
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_descriptors_fpfh (new pcl::PointCloud<pcl::FPFHSignature33> ());
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_descriptors_fpfh (new pcl::PointCloud<pcl::FPFHSignature33> ());
  // PFH descriptors
  pcl::PointCloud<pcl::PFHSignature125>::Ptr model_descriptors_pfh (new pcl::PointCloud<pcl::PFHSignature125> ());
  pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_descriptors_pfh (new pcl::PointCloud<pcl::PFHSignature125> ());
  // PFHRGB descriptors
  //pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr model_descriptors_pfhrgb (new pcl::PointCloud<pcl::PFHRGBSignature250> ());
  //pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr scene_descriptors_pfhrgb (new pcl::PointCloud<pcl::PFHRGBSignature250> ());
  std::string source;

  //
  //  Load clouds
  //
  if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }
  if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }

  //
  //  Set up resolution invariance
  //
  if (use_cloud_resolution_)
  {
    float resolution = static_cast<float> (computeCloudResolution (model));
    if (resolution != 0.0f)
    {
      model_ss_   *= resolution;
      scene_ss_   *= resolution;
      rf_rad_     *= resolution;
      descr_rad_  *= resolution;
      cg_size_    *= resolution;
    }

    std::cout << "Model resolution:       " << resolution << std::endl;
    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
    std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
    std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
    std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
    std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
  }


  //
  //  Downsample Clouds to Extract keypoints 
  //
  
  if (uniform)
  {
    source = "model";
    uniform_sampling(model,model_keypoints,1);
    source = "scene";
    uniform_sampling(scene,scene_keypoints,0);
  }
  
  else if (iss)
  {
    source = "model";
    compute_iss(model,model_keypoints, source); 
    source = "scene"; 
    compute_iss(scene,scene_keypoints, source); 
  } 

  else if (harris)
  {
   
    source = "model";
    compute_harris(model,model_keypoints, source);
    source = "scene";
    compute_harris(scene,scene_keypoints, source);
  }
  
  else if (sift_k)
  {

    source = "model";
    compute_sift(model,model_keypoints, source);
    source = "scene";
    compute_sift(scene,scene_keypoints, source);
  }
  
  if(model_keypoints->size()==0||scene_keypoints->size()==0) 
    {
        pcl::console::print_warn ("\nNo keypoints  !!\n");
        return 0;
    }
  
  //
  //  Compute Normals
  //
  if (ShotOMP)
  {
  
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (model);
    norm_est.compute (*model_normals);

    norm_est.setInputCloud (scene);
    norm_est.compute (*scene_normals);
  }
  
  if (normal_3d)
  {
  
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setRadiusSearch(0.03);
    norm_est.setInputCloud (model);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    norm_est.setSearchMethod(kdtree);
    norm_est.compute (*model_normals);
    
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est2;
    norm_est2.setRadiusSearch(0.03);
    norm_est2.setInputCloud (scene);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointXYZ>);
    norm_est2.setSearchMethod(kdtree2);
    norm_est2.compute (*scene_normals);
  }
  
  
  //
  //  Compute Descriptor for keypoints - different descriptor types
  //
  if (shot)
  {
    source = "model";
    compute_shot(model_keypoints,model,model_normals,model_descriptors_SHOT, source); 
    source = "scene";
    compute_shot(scene_keypoints,scene,scene_normals,scene_descriptors_SHOT, source);
    if(model_descriptors_SHOT->size()>0 && scene_descriptors_SHOT->size()>0)
        find_match_shot(model_descriptors_SHOT,scene_descriptors_SHOT,model_scene_corrs); 
  }
  
  else if (pfh)
  {
    source = "model";
    compute_pfh(model_keypoints,model,model_normals,model_descriptors_pfh, source);
    source = "scene";
    compute_pfh(scene_keypoints,scene,scene_normals,scene_descriptors_pfh, source);
    
    if(model_descriptors_pfh->size()>0 && scene_descriptors_pfh->size()>0)
        find_match_pfh(model_descriptors_pfh,scene_descriptors_pfh,model_scene_corrs); 
  }
  
  else if (fpfh)
  {   
    source = "model";
    compute_fpfh(model_keypoints,model,model_normals,model_descriptors_fpfh, source);  
    source = "scene"; 
    compute_fpfh(scene_keypoints,scene,scene_normals,scene_descriptors_fpfh, source); 
    if(model_descriptors_fpfh->size()>0 && scene_descriptors_fpfh->size()>0)
        find_match_fpfh(model_descriptors_fpfh,scene_descriptors_fpfh,model_scene_corrs); 
  }
  
  
  
  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  //  Using Hough3D
  if (use_hough_)
  {
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else // Using GeometricConsistency
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  //
  //  Output results
  //
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }

  //
  //  Visualization
  //
  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  viewer.addPointCloud (scene, "scene_cloud");

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  if (show_correspondences_ || show_keypoints_)
  {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints_)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }
  float centroidData[rototranslations.size ()][3]; //
  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());
    
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*rotated_model, centroid);
    std::cout << "The XYZ coordinates of the centroid are: ("
			  << centroid[0] << ", "
			  << centroid[1] << ", "
			  << centroid[2] << ")." << std::endl;
    
    for (int l = 0; l<3; l++) //
    {
    	centroidData[i][l] = centroid[l];
    }
    
    if (show_correspondences_)
    {
      for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }
  std::cout << "Saving to file" << std::endl;
  std::ofstream out("centroidData.csv");
  for (int i = 0; i < rototranslations.size (); i++)
  {
        for (int j = 0; j < 3; j++)
            out << centroidData[i][j] << ',';
        out << "\n";
  }
  out.close();
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);
}

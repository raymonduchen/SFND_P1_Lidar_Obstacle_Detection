// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

#include <unordered_set>

#include "cluster/euclidean_cluster.h"
#include "ransac/ransac3d.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  typename pcl::PointCloud<PointT>::Ptr voxel_grid_filtered{new pcl::PointCloud<PointT>};
    
  // Step #1 - VoxelGrid quantization
  typename pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);  // Units in metres
  vg.filter(*voxel_grid_filtered);

  // Step #2 - Crop to the region of interest
  typename pcl::PointCloud<PointT>::Ptr voxel_grid_crop{
      new pcl::PointCloud<PointT>};
  typename pcl::CropBox<PointT> crop(true);
  crop.setMin(minPoint);
  crop.setMax(maxPoint);
  crop.setInputCloud(voxel_grid_filtered);
  crop.filter(*voxel_grid_crop);

  // Remove the roof points
  std::vector<int> indices_to_remove_roof;
  const Eigen::Vector4f min_point_roof(-2, -2, -1, 1);
  const Eigen::Vector4f max_point_roof(3, 2, 1, 1);
  typename pcl::CropBox<PointT> crop_roof(true);
  crop_roof.setMin(min_point_roof);
  crop_roof.setMax(max_point_roof);
  crop_roof.setInputCloud(voxel_grid_crop);
  crop_roof.filter(indices_to_remove_roof);

  // First create an indices object that contains the indices we want to remove
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (const int idx : indices_to_remove_roof) 
    inliers->indices.push_back(idx);

  // Next, create an extraction tool that will remove the indices
  // Set the negative flag to true to remove these indices
  typename pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(voxel_grid_crop);
  extract.setIndices(inliers);
  extract.setNegative(true);  // Inliers will now remove the points instead
  extract.filter(*voxel_grid_crop);


  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return voxel_grid_crop;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>() );
  typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>() );

  for (int index : inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
  return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, 
          typename pcl::PointCloud<PointT>::Ptr> 
          ProcessPointClouds<PointT>::SegmentPlane(
            typename pcl::PointCloud<PointT>::Ptr cloud, 
            int maxIterations, 
            float distanceThreshold,
            const bool useCustom)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
    
  pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
  pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

  if (!useCustom){
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
  }
  else 
  {  
    // Using custom implementation
    std::unordered_set<int> inliers_set = RANSAC3D<PointT>::Ransac3D(cloud, maxIterations, distanceThreshold);
    for (const int index : inliers_set)
      inliers->indices.push_back(index);
  }    

  if (inliers->indices.size() ==0)
    std::cerr << "There were no inliers with this point cloud - plane extraction is not possible" << std::endl;

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, 
            typename pcl::PointCloud<PointT>::Ptr> 
            segResult = SeparateClouds(inliers,cloud);

  return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
            ProcessPointClouds<PointT>::Clustering(
                typename pcl::PointCloud<PointT>::Ptr cloud, 
                float clusterTolerance, 
                int minSize, 
                int maxSize,
                const bool useCustom)
{
  auto startTime = std::chrono::steady_clock::now();
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  if (!useCustom) {  
    // Use PCL method  
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (pcl::PointIndices getIndices: clusterIndices)
    {
      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
      for (int id : getIndices.indices)
        cloudCluster->points.push_back(cloud->points[id]);

      cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;

      clusters.push_back(cloudCluster);
    }
  }
  else
  {
    // Use custom implementation
    KdTree* tree = new KdTree;

    std::vector<std::vector<float>> points;

    for (int i = 0; i < cloud->points.size(); i++) {
      const std::vector<float> pt = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      tree->insert(pt, i);
      points.push_back(pt);
    }

    // Get the cluster IDs
    const std::vector<std::vector<int>> kdtree_clusters = euclideanCluster(points, tree, clusterTolerance);

    delete tree;
    tree = nullptr;

    // Check if cluster within the min-max range - if it is, add the cluster into clusters
    for (const auto& clust : kdtree_clusters) {
      if (clust.size() >= minSize && clust.size() <= maxSize) {
        // Create new pointer to point cloud object of type PointT
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster{new pcl::PointCloud<PointT>};

        for (const auto id : clust)
          cloud_cluster->points.push_back(cloud->points[id]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
      }
    }
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()  << " clusters" << std::endl;

  return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
  pcl::io::savePCDFileASCII (file, *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
  typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) 
    PCL_ERROR ("Couldn't read file \n");
  
  std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

  return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}
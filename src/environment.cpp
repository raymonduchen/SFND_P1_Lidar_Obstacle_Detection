/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* point_processor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
               const bool use_custom = false) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud =
      point_processor->FilterCloud(input_cloud, 0.3,
                                   Eigen::Vector4f(-10, -5.5, -3, 1),
                                   Eigen::Vector4f(30, 7, 3, 1));

  ///// Obstacle Detection 
  int maxIterations = 150; //25
  float distanceThreshold = 0.3; //0.3
  // Step #1 - Find the road plane and remove from the point cloud
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr>
      segment_cloud = point_processor->SegmentPlane(filter_cloud, maxIterations, distanceThreshold, use_custom);

  // Step #2 - Perform clustering
  float clusterTolerance = 1.3;  //0.4
  int minSize = 10; //10
  int maxSize = 300; //500
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters =
      point_processor->Clustering(segment_cloud.first, clusterTolerance, minSize, maxSize, use_custom);

  // Render plane cloud
  renderPointCloud(viewer, segment_cloud.second, "planeCloud", Color(0, 1, 0));

  // Step #3 - Place bounding boxes around objects
  int cluster_id = 0;
  std::vector<Color> colors = {Color(1, 1, 0), Color(0, 1, 1), Color(1, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloud_clusters) {
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(cluster_id),
                     colors[cluster_id % colors.size()]);

    // Use standard bounding box
    Box box = point_processor->BoundingBox(cluster);
    renderBox(viewer, box, cluster_id);
    ++cluster_id;
  }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //CameraAngle setAngle = XY;
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    bool useCustom = true;

    // Make point processor outside of the loop
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream =
        pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped()) {
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

      cityBlock(viewer, pointProcessorI, inputCloudI, useCustom);

      streamIterator++;
      if (streamIterator == stream.end()) { streamIterator = stream.begin(); }

      viewer->spinOnce();
    }
    
}
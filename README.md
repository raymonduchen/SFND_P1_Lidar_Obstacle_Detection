# LiDAR Obstacle Detection

<img src="media/result.gif" width="700" height="400" />

The project demonstrates the full pipeline from streaming point cloud data,  filtering cloud, segment ground plane, clustering objects, and rendering bounding boxes around clustered objects.

Point cloud data is collected from a lidar set on top of a vehicle (ego vehicle) when it drives in city. X axis of lidar is along vehicle heading direction, Y axis is on the left side, and Z axis is towards the top side.

This project is also the first project from [Udacity sensor fusion nanodegree (SFND)](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313).



## Usage

- Prerequisite 
  - PCL (Point Cloud Library) : See https://github.com/udacity/SFND_Lidar_Obstacle_Detection for installation.

```
$> git clone https://github.com/raymonduchen/SFND_P1_Lidar_Obstacle_Detection
$> cd SFND_P1_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```



## Pipeline of the project

The main function is in `environment.cpp`, and it uses functions in `processPointClouds.cpp` to implement the following pipeline :

- **Stream Point Cloud**: stream point cloud from point cloud data files (.pcd)

- **Filter Cloud** : downsample point cloud and remove non necessary parts
  - **Voxel grid filtering** : downsample using voxel grid filtering method
  - **Crop to region of interest** : 
    - Region of interest : X = (+30, -10), Y = (7, -5.5), Z = (-3, 3)
  - **Remove the roof points** : remove roof points from ego vehicle
- **Segment Ground Plane** : use self-implemented RANSAC function to segment gound plane. The resultant point cloud becomes two point clouds : ground plane point cloud and objects point cloud.

- **Clustering Objects** : for objects point cloud, use self-implemented Euclidean clustering function to cluster objects. The resultant point clouds will be several clustered point clouds represented different objects in reality (such as vehicle, pedestrian, pole, etc.)
- **Render Bounding Box** : render enclosed bounding box around each clustered point clouds.



## File directory

```
SFND_P1_Lidar_Obstacle_Detection
|- CMakeList.txt
|- media
|  |- result.gif
|- README.md
|- src
   |- environment.cpp                  // main 
   |- processPointClouds.cpp           //
   |- processPointClouds.h
   |- cluster
   |  |- euclidean_cluster.cpp         // self-implemented Euclidean clustering
   |  |- euclidean_cluster.h           
   |  |- kdtree.h                      // self-implemented kdtree
   |- ransac
   |  |- ransac3d.h                    // self-implemented ransac
   |- render
   |  |- box.h
   |  |- render.cpp
   |  |- render.h
   |- sensors
      |- lidar.h
      |- data
         |- pcd
            |- data_1
            |  |- 0000000000.pcd
            |  |- 0000000001.pcd               
            |  |- ...
            |  |- 0000000021.pcd
            |- data_2
            |  |- 0000000000.pcd
            |  |- 0000000001.pcd               
            |  |- ...
            |  |- 0000000053.pcd
            |- simpleHighway.pcd   
```
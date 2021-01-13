#ifndef _RANSAC_3D_H_
#define _RANSAC_3D_H_

#include <pcl/common/common.h>
#include <unordered_set>

template <typename PointT>
class RANSAC3D {
 public:
  static std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, const int maxIterations, const float distanceTol) 
  {  
    std::unordered_set<int> inlier_results;
    std::unordered_set<int> proposed_inliers;
    const int num_points = cloud->points.size();
    srand(time(NULL));

    for (int i = 0; i < maxIterations; i++) {
      proposed_inliers.clear();

      std::unordered_set<int> indices;
      while (indices.size() != 3)
        indices.insert(std::rand() % num_points);

      std::unordered_set<int>::iterator it = indices.begin();
      const auto& pt1 = cloud->points[*it++];
      const auto& pt2 = cloud->points[*it++];
      const auto& pt3 = cloud->points[*it];

      const double A = (pt2.y - pt1.y) * (pt3.z - pt1.z) - (pt2.z - pt1.z) * (pt3.y - pt1.y);
      const double B = (pt2.z - pt1.z) * (pt3.x - pt1.x) - (pt2.x - pt1.x) * (pt3.z - pt1.z);
      const double C = (pt2.x - pt1.x) * (pt3.y - pt1.y) - (pt2.y - pt1.y) * (pt3.x - pt1.x);
      const double D = -(A * pt1.x + B * pt1.y + C * pt1.z);

      for (int j = 0; j < num_points; j++) {
        const auto& pt = cloud->points[j];
        const double dist = std::abs(A * pt.x + B * pt.y + C * pt.z + D) / std::sqrt(A * A + B * B + C * C);
        if (dist <= distanceTol)
          proposed_inliers.insert(j);
      }

      if (proposed_inliers.size() > inlier_results.size()) {
        inlier_results = proposed_inliers;
      }
    }

    return inlier_results;
  };
};
#endif
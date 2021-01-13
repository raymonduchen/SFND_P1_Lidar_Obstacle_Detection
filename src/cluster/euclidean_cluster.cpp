#include "kdtree.h"

void Proximity(const int id,
               const std::vector<std::vector<float>>& points,
               KdTree* tree,
               std::vector<int>& cluster,
               std::vector<bool>& processed,
               const float distanceTol) {

  const std::vector<float> point = points[id];
  processed[id] = true;
  cluster.push_back(id);

  const std::vector<int> nearby_ids = tree->search(point, distanceTol);

  for (const int id : nearby_ids) {
    if (!processed[id]) {
      Proximity(id, points, tree, cluster, processed, distanceTol);
    }
  }
}

std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>>& points,  
    KdTree* tree,    
    float distanceTol) { 

  std::vector<std::vector<int>> clusters;
  std::vector<bool> processed(points.size(), false);

  for (size_t i = 0; i < points.size(); i++) {
    if (!processed[i]) {
      std::vector<int> cluster;  
      Proximity(i, points, tree, cluster, processed, distanceTol);
      clusters.push_back(cluster);
    }
  }

  return clusters;
}

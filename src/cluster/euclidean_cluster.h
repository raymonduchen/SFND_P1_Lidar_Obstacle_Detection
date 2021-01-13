#ifndef __EUCLIDEAN_CLUSTER_H__
#define __EUCLIDEAN_CLUSTER_H__
#include "kdtree.h"

void Proximity(const int id,
	           const std::vector<std::vector<float>>& points,
	           KdTree* tree,
	           std::vector<int>& cluster,
	           std::vector<bool>& processed,
	           const float distanceTol);

std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>>& points,  
    KdTree* tree,       
    float distanceTol); 

#endif
#include "../render/render.h"

// Structure to represent node of kd tree
struct Node
{
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;

  Node(std::vector<float> arr, int setId)
  : point(arr), id(setId), left(NULL), right(NULL)
  {}
};

struct KdTree
{
  Node* root;

  KdTree()
  : root(NULL)
  {}

  void insertHelper (Node** node, uint depth, std::vector<float> point, int id)
  {
    if (*node==NULL)
      *node = new Node(point, id);
    else
    {
      uint cd = depth % 2;

      if (point[cd] < ((*node)->point[cd]))
        insertHelper(&((*node)->left), depth+1, point, id);
      else
        insertHelper(&((*node)->right), depth+1, point, id);
    }
  }

  void insert(std::vector<float> point, int id)
  {
    insertHelper(&root, 0, point, id);
  }
  
  void searchHelper(std::vector<int>& ids,
                    const std::vector<float>& target,
                    const float& distanceTol,
                    const Node* node,
                    const int depth) 
  {
    if (node == nullptr)
      return; 

    const int index = depth % target.size();
    const float coord_node = node->point[index];
    const float coord_target = target[index];

    // Check if all coordinates within the tolerance
    bool check = true;
    for (size_t i = 0; i < target.size(); i++)
      check &= (std::abs(node->point[index] - target[index]) <= distanceTol);

    if (check) {
      // Calculate distance if within bounding box
      double dist = 0.0;
      for (size_t i = 0; i < target.size(); i++)
        dist += (node->point[i] - target[i]) * (node->point[i] - target[i]);

      if (dist <= (distanceTol * distanceTol))
        ids.push_back(node->id);
    }

    if ((coord_target - distanceTol) < coord_node)
      searchHelper(ids, target, distanceTol, node->left, depth + 1);

    if ((coord_target + distanceTol) > coord_node)
      searchHelper(ids, target, distanceTol, node->right, depth + 1);
  }

  std::vector<int> search(std::vector<float> target, float distanceTol)
  {
    std::vector<int> ids;

    searchHelper(ids, target, distanceTol, root, 0);
    return ids;
  }
  

};


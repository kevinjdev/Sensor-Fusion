/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

namespace mytree
{
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	
  	void insertNode(Node *&node, int depth, std::vector<float> point, int id)
    {
      if(node == NULL)
      {
          node = new Node(point, id);
      }
      else {
        // if current depth is 0, then point index represents x axis
        // if current depth is 1, then point index represents y axis
        int current_depth = depth % 2;

        if(point[current_depth] < node->point[current_depth])
        {
            insertNode(node->left, depth + 1, point, id);
        }
        else
        {
            insertNode(node->right, depth + 1, point, id);
        }
      }
    }
	void insert(std::vector<float> point, int id)
	{
      insertNode(root, 0, point, id);
   	}
  	
  	void searchNodes(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int>& ids)
    {
    	if (node == NULL)
        {
          return;
        }

        if ( (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol))
        &&   (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) )
        {
          float x_dist = node->point[0] - target[0];
          float y_dist = node->point[1] - target[1];
          float distance = sqrt(x_dist * x_dist + y_dist * y_dist);
          if (distance <= distanceTol)
          {
            ids.push_back(node->id);
          }
        }
      
      	// Explore further down tree
      	if ((target[depth % 2] - distanceTol) < node->point[depth % 2])
        {
          searchNodes(target, node->left, depth + 1, distanceTol, ids);
        }
      	if ((target[depth % 2] + distanceTol) > node->point[depth % 2])
            {
              searchNodes(target, node->right, depth + 1, distanceTol, ids);
            }	
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
      	searchNodes(target, root, 0, distanceTol, ids);
		return ids;
	}
};
}




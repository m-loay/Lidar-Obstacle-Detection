/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	Node*getNewNode(std::vector<float> point, int id)
	{
		return new Node(point,id);
	}

    void insertNodeHelper (Node *&node,uint depth ,const std::vector<float>& point, int id)
	{
		if(node == NULL)
		{
			node = getNewNode(point,id);
		}
		else
		{
			uint cd = depth % node->point.size();
			if(point[cd] < node->point[cd])
			{
				insertNodeHelper(node->left, depth+1 ,point ,id);
			}
			else
			{
				insertNodeHelper(node->right, depth+1, point, id);
			}
		}
	}

	void insert(const std::vector<float> &point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		Node* &node_ref =  root;
		insertNodeHelper(node_ref,0,point,id);
	}

	bool check_distance_tolernace(const std::vector<float> &node_point ,
	                              const std::vector<float> &target_point,
								  const float distanceTol)
	{
		bool valid = true;
		int delta = 0;
		float sqauredDistance = 0.0;

		for(int counter=0 ; counter< target_point.size(); counter++)
		{
			valid = valid &&
			        (node_point[counter] < (target_point[counter] + distanceTol))&&
			        (node_point[counter] > (target_point[counter] - distanceTol));

			delta = node_point[counter] - target_point[counter];
			sqauredDistance +=  pow(delta,2.0);
		}

		valid = valid && (sqrt(sqauredDistance)<=distanceTol);
		return valid;
	}

	void searchNodeHelper(std::vector<int> &ids, const std::vector<float> &target,
				const float distanceTol, Node* node, int depth) 
	{
		if (node != NULL) 
		{
			if (check_distance_tolernace(node->point, target, distanceTol)) 
			{
				ids.push_back(node->id);
			}

             uint cd = depth % target.size();
			if ((target[cd]-distanceTol) < node->point[cd]) 
			{
				searchNodeHelper(ids, target, distanceTol, node->left, depth+1);
			}

			if ((target[cd]+distanceTol) > node->point[cd]) 
			{
				searchNodeHelper(ids, target, distanceTol, node->right, depth+1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float> &target, const float distanceTol)
	{
		std::vector<int> ids;
		searchNodeHelper(ids, target, distanceTol, root, 0);
		return ids;
	}
};





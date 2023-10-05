#include "render/render.h"

//Updated version.
struct Node
{
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId)
        : point(arr), id(setId), left(NULL), right(NULL)
    {
    }
};

struct KdTree
{
    Node *root;

    KdTree()
        : root(NULL)
    {
    }

    // Finds an appropriate spot in the kd-tree to insert node.
    void InsertHelper(Node *&node, std::vector<float> point, int id, int depth)
    {

        // base case
        if (node == NULL)
        {
            node = new Node(point, id);
            return;
        }

        // recursive case

        // Comparison of x or y (or others) value depends on depth of tree
        uint coordIdx = depth % 3; // For 3D (x-axis = 0, y-axis = 1, z-axis = 3)

        if (point[coordIdx] < (node->point[coordIdx]))
        {
            InsertHelper(node->left, point, id, depth + 1);
        }

        else
        {
            InsertHelper(node->right, point, id, depth + 1);
        }
    }

    void insert(std::vector<float> point, int id)
    {
        // Insert a new point into the tree
        // the function should create a new node and place correctly with in the root

        InsertHelper(root, point, id, 0);
    }

    // Return a list of point ids in the tree that are within distance of target.
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        SearchHelper(root, target, distanceTol, 0, ids);
        return ids;
    }

    // Recursive helper function for search
    void SearchHelper(Node *&node, std::vector<float> target, float distanceTol, int depth, std::vector<int> &ids)
    {
        // Base case
        if (node == NULL)
        {
            return;
        }

        // Check if node is within the bounding box
        if (IsWithinBoundingBox(node->point, target, distanceTol))
        {
            // Add node's ID to the results
            ids.push_back(node->id);
        }

        // Recursively search left and right subtrees
        SearchHelper(node->left, target, distanceTol, depth + 1, ids);
        SearchHelper(node->right, target, distanceTol, depth + 1, ids);
    }

    // Checks if a point is within a bounding box
    bool IsWithinBoundingBox(std::vector<float> point, std::vector<float> target, float distanceTol)
    {
        for (size_t i = 0; i < point.size(); i++)
        {
            if (point[i] > (target[i] + distanceTol) || point[i] < (target[i] - distanceTol))
            {
                return false;
            }
        }

        return true;
    }
};

#include <pcl/impl/point_types.hpp>
#include <vector>

struct Node {
    pcl::PointXYZI point;
    int id;
    Node *left;
    Node *right;

    Node(pcl::PointXYZI point, int setId) : point(point), id(setId), left(NULL), right(NULL) {}
};


struct KdTree {
    Node *root;

    KdTree()
            : root(NULL) {}

    void insertP(Node **node, uint depth, pcl::PointXYZI point, uint id) {
        if (*node == NULL)
            *node = new Node(point, id);
        else {
            switch (depth % 3) {
                case 0:
                    if (point.x < ((*node)->point.x))
                        insertP(&((*node)->left), depth + 1, point, id);
                    else
                        insertP(&((*node)->right), depth + 1, point, id);
                    break;
                case 1:
                    if (point.y < ((*node)->point.y))
                        insertP(&((*node)->left), depth + 1, point, id);
                    else
                        insertP(&((*node)->right), depth + 1, point, id);
                    break;
                case 2:
                    if (point.z < ((*node)->point.z))
                        insertP(&((*node)->left), depth + 1, point, id);
                    else
                        insertP(&((*node)->right), depth + 1, point, id);
                    break;
            }
        }
    }

    void insert(pcl::PointXYZI point, int id) {
        insertP(&root, 0, point, id);
    }

    float dist(pcl::PointXYZI p1, pcl::PointXYZI p2){
        float dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                          (p1.y - p2.y) * (p1.y - p2.y) +
                          (p1.y - p2.z) * (p1.z - p2.z));
        return dist;
    }

    void searchP(pcl::PointXYZI target, Node *node, int depth, float distanceTol, std::vector<int> &ids) {
        if (node != NULL) {
            float minx = target.x - distanceTol;
            float maxx = target.x + distanceTol;
            float miny = target.y - distanceTol;
            float maxy = target.y + distanceTol;
            float minz = target.z - distanceTol;
            float maxz = target.z + distanceTol;

            if (node->point.x >= minx && node->point.x <= maxx &&
                node->point.y >= miny && node->point.y <= maxy &&
                node->point.z >= minz && node->point.z <= maxz &&
                dist(node->point, target) <= distanceTol){
                    ids.push_back(node->id);
            }

            switch (depth % 3) {
                case 0:
                    if ((target.x - distanceTol) < node->point.x)
                        searchP(target, node->left, depth + 1, distanceTol, ids);
                    if ((target.x + distanceTol) > node->point.x)
                        searchP(target, node->right, depth + 1, distanceTol, ids);
                    break;
                case 1:
                    if ((target.y - distanceTol) < node->point.y)
                        searchP(target, node->left, depth + 1, distanceTol, ids);
                    if ((target.y + distanceTol) > node->point.y)
                        searchP(target, node->right, depth + 1, distanceTol, ids);
                    break;
                case 2:
                    if ((target.z - distanceTol) < node->point.z)
                        searchP(target, node->left, depth + 1, distanceTol, ids);
                    if ((target.z + distanceTol) > node->point.z)
                        searchP(target, node->right, depth + 1, distanceTol, ids);
                    break;

            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(pcl::PointXYZI target, float distanceTol) {
        std::vector<int> ids;
        searchP(target, root, 0, distanceTol, ids);
        return ids;
    }


};





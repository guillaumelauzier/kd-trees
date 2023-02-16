# kd-trees

Here is an example implementation of a kd-tree in C++ using the recursive construction algorithm:

```
#include <iostream>
#include <vector>
#include <algorithm>

// 3D point class
class Point3D {
public:
    double x, y, z;
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

// kd-tree node class
class KDNode {
public:
    Point3D point;
    KDNode* left;
    KDNode* right;

    KDNode(Point3D point_) : point(point_), left(nullptr), right(nullptr) {}
    ~KDNode() {
        delete left;
        delete right;
    }
};

// kd-tree class
class KDTree {
public:
    KDTree(std::vector<Point3D>& points) {
        root = buildTree(points, 0);
    }

    ~KDTree() {
        delete root;
    }

    // search for the nearest neighbor to a given point
    Point3D nearestNeighbor(Point3D query) {
        double bestDist = std::numeric_limits<double>::max();
        Point3D bestPoint = root->point;
        searchNearest(root, query, &bestDist, &bestPoint);
        return bestPoint;
    }

private:
    KDNode* root;

    // recursive function to build a kd-tree
    KDNode* buildTree(std::vector<Point3D>& points, int depth) {
        if (points.empty()) {
            return nullptr;
        }
        int dim = depth % 3;
        if (points.size() == 1) {
            return new KDNode(points[0]);
        }
        std::sort(points.begin(), points.end(), [dim](Point3D a, Point3D b) {
            if (dim == 0) return a.x < b.x;
            if (dim == 1) return a.y < b.y;
            return a.z < b.z;
        });
        int median = points.size() / 2;
        KDNode* node = new KDNode(points[median]);
        node->left = buildTree(std::vector<Point3D>(points.begin(), points.begin() + median), depth + 1);
        node->right = buildTree(std::vector<Point3D>(points.begin() + median + 1, points.end()), depth + 1);
        return node;
    }

    // recursive function to search for the nearest neighbor
    void searchNearest(KDNode* node, Point3D query, double* bestDist, Point3D* bestPoint) {
        if (!node) {
            return;
        }
        double dist = (node->point.x - query.x) * (node->point.x - query.x)
                    + (node->point.y - query.y) * (node->point.y - query.y)
                    + (node->point.z - query.z) * (node->point.z - query.z);
        if (dist < *bestDist) {
            *bestDist = dist;
            *bestPoint = node->point;
        }
        int dim = 0;
        if (query.y > node->point.y && query.y > node->point.z) {
            dim = 0;
        } else if (query.x > node->point.x && query.x > node->point.z) {
            dim = 1;
        } else {
            dim = 2;
        }
        if (query[dim] < node->point[dim]) {
            searchNearest(node->left, query, bestDist, bestPoint);
            if (query[dim] + *bestDist >= node->point[dim]) {
                searchNearest(node->right, query, bestDist, bestPoint);
        } else {
            searchNearest(node->right, query, bestDist, bestPoint);
            if (query[dim] - *bestDist <= node->point[dim]) {
                searchNearest(node->left, query, bestDist, bestPoint);
            }
        }
    }
};

int main() {
    // create a vector of 3D points
    std::vector<Point3D> points = {
        Point3D(2.0, 3.0, 4.0),
        Point3D(5.0, 1.0, 7.0),
        Point3D(6.0, 8.0, 2.0),
        Point3D(9.0, 4.0, 3.0),
        Point3D(1.0, 2.0, 6.0)
    };
    
    // build a kd-tree from the points
    KDTree tree(points);

    // search for the nearest neighbor to a query point
    Point3D query(4.0, 5.0, 3.0);
    Point3D nearest = tree.nearestNeighbor(query);
    std::cout << "Nearest neighbor to (" << query.x << ", " << query.y << ", " << query.z << ") is (" 
              << nearest.x << ", " << nearest.y << ", " << nearest.z << ")" << std::endl;

    return 0;
}
```

This implementation creates a KDTree object using a vector of Point3D objects, and then searches for the nearest neighbor to a query point using the nearestNeighbor function. The nearestNeighbor function uses a recursive search algorithm to traverse the kd-tree and find the closest point to the query point.

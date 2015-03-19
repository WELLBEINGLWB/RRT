#ifndef _MOTION_PLAN_H_
#define _MOTION_PLAN_H_

#include <iostream>
#include <fstream>
#include <ostream>

#include <iterator>
#include <vector>
#include <string>

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <unistd.h>

typedef struct{
  double x;
  double y;
  double z;
} POINT;

typedef struct {
  double xrange[2];
  double yrange[2];
  double zrange[2];
} RANGE;


namespace MotionPlan
{

  // 対象の点が障害物に衝突していないかを判定
  bool clear(const double* xMin, const double* xMax,
             const double* yMin, const double* yMax,
             const double* zMin, const double* zMax,
             int numObstacles,
             double xTest, double yTest, double zTest);

  // 障害物の各頂点を抽出
  void CreateGridPoint(const double* xMin, const double* xMax,
                       const double* yMin, const double* yMax,
                       const double* zMin, const double* zMax, POINT *P, int i);

  // 平面の方程式を導出
  void PlaneEquation(POINT p[], int i0[], int i1[], int i2[], int i, double a[]);

  // 与えられた2点の各座標値の大きさを比較
  void Pcompare(POINT A, POINT B, POINT *compare);



  // (xStart,yStart,zStart) と (xDest,yDest,zDest) を結ぶ直線と障害物が干渉しているか判定
  bool link(const double* xMin, const double* xMax,
            const double* yMin, const double* yMax,
            const double* zMin, const double* zMax,
            int numObstacles,
            double xStart, double yStart, double zStart,
            double xDest, double yDest, double zDest);


  void CreateGrid(const double xMin, const double xMax,
                  const double yMin, const double yMax,
                  const double zMin, const double zMax, POINT *P);

  void PlaneEquation(POINT p[], int i0[], int i1[], int i2[], int i, double a[]);

  void Pcompare(double xStart, double yStart, double zStart,
                double xDest, double yDest, double zDest, POINT *compare);

  bool Intersection(const double xMin, const double xMax,
                    const double yMin, const double yMax,
                    const double zMin, const double zMax,
                    double xStart, double yStart, double zStart,
                    double xDest, double yDest, double zDest);


  class RRT
  {
  public:

    /// Represents a node in an RRT
    /// Stores its own positional information, as well as
    /// a list of its children and a pointer to its parent
    class TreeNode
    {
    public:
      double x, y, z;
      std::vector<TreeNode*> children;
      TreeNode* parent;

      /// Used for outputting the tree
      int nodeID;

      /// Given a point (sampleX, sampleY), find the nearest node within
      /// this node's subtree (including itself), and place the distance between
      /// that node and the point in distance
      TreeNode* nearestNode(double sampleX, double sampleY, double samplez, double* distance);

      /// Delete this node's subtree from memory (excluding itself, of course)
      void deleteChildren();
    };

    /// Basic structure used to encapsulate two nodes into an edge.
    /// Used only for outputting the tree
    class Edge
    {
    public:
      int node1; int node2;

      Edge(int n1, int n2) : node1(n1), node2(n2) {}
    };

    /// Initializes the RRT with the values given, as well as seeds the
    /// random number generator with the system time.
    RRT(double* xMin, double* xMax, double* yMin, double* yMax, double* zMin, double* zMax, int numObs,
        double xL, double xR, double yT, double yB, double zT, double zB,
        double xS, double yS, double zS,
        double xG, double yG, double zG,
        double step);

    /// Initializes the RRT as above, except uses data from a file to do so.
    RRT(std::string fileName);

    /// Destructor
    ~RRT();

    /// Initializes (or re-initializes) the RRT using a data file
    void initFromFile(std::string fileName);

    /// Main pathfinding function of the RRT.
    /// If, a path is found between start and goal within MAX_ITERATIONS,
    /// the sequence of node IDs is placed in nodePath, the number of iterations performed
    /// is placed in iterations, and the length of the path is in pathLength.
    ///
    /// As well, the attributes 'nodes' and 'edges' are populated with the tree
    /// generated by this function, for output purposes.
    bool findPath(int* iterations, int* nodePath, int* pathLength);

    /// Generates a point (x,y) within the confines of the space and places
    /// its coordinates in 'x' and 'y'.
    void randFreeSample(double* x, double* y, double* z);

    /// Given a point (x,y), returns a pointer to the TreeNode in the RRT
    /// closest to that point.
    TreeNode* nearestNode(double x, double y, double z);

    /// Given a pointer to the nearest TreeNode in the RRT to a point (x,y),
    /// will attempt to create a new TreeNode of distance = stepSize from
    /// nearest in the direction of (x,y). If this node can't be created
    /// (i.e. there's an obstacle between nearest and this new node), this
    /// function returns NULL. If the new node is valid, a pointer to it is
    /// returned.
    TreeNode* genNewNode(const TreeNode* nearest, double x, double y, double z);

    /// Checks whether the goal position is within stepSize distance and
    /// reachable from checkNode. If so, return true. If it's too far away
    /// or if link() between checkNode and the goal fails, return false.
    bool checkGoal(const TreeNode* checkNode);


    void CreateCube(std::ostream &cube);
    /// Write out information about the tree into outStream.
    /// Displays the RRT's nodes and edges.
    void outputTree(std::ostream& outStream);
    void outputTree(FILE *outStream);
    void gnuplot_config(FILE *outStream);

    /// Pointer to the root (start) node of the RRT.
    TreeNode* root;

    /// Min/Max coordinates of all obstacles in space.
    double* xMin;
    double* xMax;
    double* yMin;
    double* yMax;
    double* zMin;
    double* zMax;

    /// Number of obstacles in space.
    int numObstacles;

    /// Start position in space
    double xStart;
    double yStart;
    double zStart;

    /// Goal position in space
    double xGoal;
    double yGoal;
    double zGoal;

    /// Max. distance toward each sampled position we
    /// should grow our tree
    double stepSize;

    /// Boundaries of the space
    double xLeft;
    double xRight;
    double yTop;
    double yBottom;
    double zTop;
    double zBottom;

    /// List of nodes in RRT (for output)
    std::vector<TreeNode*> nodes;

    /// List of edges in RRT (for output)
    std::vector<Edge> edges;

    /// Maximum number of iterations to run when finding a path
    /// before givin up.
    static const int MAX_ITERATIONS = 25000;
  };
};

#endif

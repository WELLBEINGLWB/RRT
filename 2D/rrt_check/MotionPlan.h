#ifndef _MOTION_PLAN_H_
#define _MOTION_PLAN_H_

#include <iostream>
#include <ostream>
#include <fstream>
#include <string>

#include <vector>
#include <iterator>

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <unistd.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// typedef struct{
//   double x;
//   double y;
//   double z;
// } POINT;

typedef struct {
  double xrange[2];
  double yrange[2];
  double zrange[2];
} RANGE;



namespace MotionPlan
{
  /// Checks whether a point (xTest,yTest) is in collision
  /// with any of the obstacles defined by their min/max coordinates.
  bool clear(const double* xMin, const double* xMax,
    const double* yMin, const double* yMax,
    int numObstacles,
    double xTest, double yTest);

  /// A geometrically exact query for whether the line between
  /// points (xStart,yStart) and (xDest,yDest) collides with any
  /// of the obstacles defined by their mein/max coordinates
  bool link(const double* xMin, const double* xMax,
   const double* yMin, const double* yMax,
   int numObstacles,
   double xStart, double yStart,
   double xDest, double yDest);

  /// Used in link for dividing cases based on whether the line
  /// between them is of zero, infinite, or general slope
  enum LineType {HORIZONTAL, VERTICAL, GENERAL};

  /// Used in equality check between two double values
  const double EPSILON = 1e-9;

  /// Check whether two double values are equal
  bool equal(double x, double y);

  class RRT
  {
  public:

    /// Represents a node in an RRT
    /// Stores its own positional information, as well as
    /// a list of its children and a pointer to its parent
    class TreeNode
    {
    public:
      double x, y;
      std::vector<TreeNode*> children;
      TreeNode* parent;

      /// Used for outputting the tree
      int nodeID;

      /// Given a point (sampleX, sampleY), find the nearest node within
      /// this node's subtree (including itself), and place the distance between
      /// that node and the point in distance
      TreeNode* nearestNode(double sampleX, double sampleY, double* distance);

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
    RRT(double* xMin, double* xMax, double* yMin, double* yMax, int numObs,
     double xL, double xR, double yT, double yB,
     double xS, double yS,
     double xG, double yG,
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
    //void my_mouse_callback(int event, int x, int y, int flags, void* param);
    //bool click_flag = false;
    bool findPath(int* iterations, int* nodePath, int* pathLength);

    /// Generates a point (x,y) within the confines of the space and places
    /// its coordinates in 'x' and 'y'.
    void randFreeSample(double* x, double* y);

    /// Given a point (x,y), returns a pointer to the TreeNode in the RRT
    /// closest to that point.
    TreeNode* nearestNode(double x, double y);

    /// Given a pointer to the nearest TreeNode in the RRT to a point (x,y),
    /// will attempt to create a new TreeNode of distance = stepSize from
    /// nearest in the direction of (x,y). If this node can't be created
    /// (i.e. there's an obstacle between nearest and this new node), this
    /// function returns NULL. If the new node is valid, a pointer to it is
    /// returned.
    TreeNode* genNewNode(const TreeNode* nearest, double x, double y);

    /// Checks whether the goal position is within stepSize distance and
    /// reachable from checkNode. If so, return true. If it's too far away
    /// or if link() between checkNode and the goal fails, return false.
    bool checkGoal(const TreeNode* checkNode);

    /// Write out information about the tree into outStream.
    /// Displays the RRT's nodes and edges.
    void outputTree(std::ostream& outStream);
    void outputTree(std::ostream &outStream, int iterations);
    void outputTree(int iterations);
    void outputSample(std::string Filemame, double x, double y);

    /// Pointer to the root (start) node of the RRT.
    TreeNode* root;

    /// Min/Max coordinates of all obstacles in space.
    double* xMin;
    double* xMax;
    double* yMin;
    double* yMax;

    /// Number of obstacles in space.
    int numObstacles;

    /// Start position in space
    double xStart;
    double yStart;

    /// Goal position in space
    double xGoal;
    double yGoal;

    /// Max. distance toward each sampled position we
    /// should grow our tree
    double stepSize;

    /// Boundaries of the space
    double xLeft;
    double xRight;
    double yTop;
    double yBottom;

    /// List of nodes in RRT (for output)
    std::vector<TreeNode*> nodes;

    /// List of edges in RRT (for output)
    std::vector<Edge> edges;

    /// Maximum number of iterations to run when finding a path
    /// before givin up.
    static const int MAX_ITERATIONS = 10000;
  };
};

#endif

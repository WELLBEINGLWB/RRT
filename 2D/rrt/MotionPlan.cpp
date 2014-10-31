#include "MotionPlan.h"
#include <cmath>
#include <iterator>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>

/// Simple iterates over each object in the space
/// and checks whether the test point lies inside the obstacle.
bool MotionPlan::clear(const double* xMin, const double* xMax,
 const double* yMin, const double* yMax,
 int numObstacles,
 double xTest, double yTest){
  for (int i = 0; i < numObstacles; ++i){
    if (xTest >= xMin[i] && xTest <= xMax[i] && yTest >= yMin[i] && yTest <= yMax[i]){
      return false;
    }
  }

  return true;
}

// First finds line between (xStart,yStart) and (xDest,yDest), called
// 'link-line'.
// Then, for each obstacle, we perform 1 of 3 different checks depending
// on whether the line is of zero, infinite, or nonzero slope.
// 最初に(xStart,yStart)と(xDest,yDest)の間の線を見つける。'link-line'と呼ぶよ。
// それぞれの障害物において、'link-line'が0、無限か有限なのかどうかの依存をチェック。


// VERTICAL/HORIZONTAL
// If either of the endpoints of link-line is inside the obstacle, collision.
// For vertical: for link-line x = a, if x is between xMin and xMax of
// the obstacle, and the endpoints of the link-line lie on either side of
// the obstacle, collision.
// For horizontal: for link-line y = a, if y is between yMin and yMax of the
// obstacle, and the endpoints of the link-line lie on either side of the
// obstacle, collision.
// Otherwise, no collision.
// 水平/垂直
// link-lineの終端のどちらかが障害物か衝突の中にある場合、
// 水平の場合、link-lineはx = aで、もしxが障害物のxMinとxMaxの範囲内やまたがっていたら...
// 垂直の場合、link-lineはy = aで、.....垂直と一緒
// その他の場合なら障害物とあたってない！


// NONZERO SLOPE
// For each side of the obstacle, we use (y-y1)=m(x-x1) to find
// the point of intersection of link-line and the line that runs
// through the side of the obstacle (called side-line).
// If that intersection point lies on the side of the obstacle, we then
// check whether our start and dest points lie on either side of side-line.
// If so, we have a collision. In any case besides the one defined here,
// there is no collision.
// 障害物のそれぞれのサイドにおいて、(y-y1)=m(x-x1)で考える。

bool MotionPlan::link(const double* xMin, const double* xMax,
  const double* yMin, const double* yMax,
  int numObstacles,
  double xStart, double yStart,
  double xDest, double yDest){
  if (!clear(xMin, xMax, yMin, yMax, numObstacles, xStart, yStart) ||
    !clear(xMin, xMax, yMin, yMax, numObstacles, xDest, yDest)){
    return false;
  }

  double dx = xDest - xStart;
  double dy = yDest - yStart;
  double m = 0.0;
  LineType lineType = GENERAL;

  if (MotionPlan::equal(dx,0))
    lineType = VERTICAL;
  else if (MotionPlan::equal(dy,0))
    lineType = HORIZONTAL;
  else
    m = dy/dx;

  for (int i = 0; i < numObstacles; ++i){
    if (lineType == VERTICAL){
      if (xDest >= xMin[i] && xDest <= xMax[i]){
       if ((yStart >= yMin[i] && yStart <= yMax[i]) ||
         (yDest >= yMin[i] && yDest <= yMax[i]) ||
         (yStart <= yMin[i] && yDest >= yMax[i]) ||
         (yDest <= yMin[i] && yStart >= yMax[i])){
         return false;
        }
      }
    }

    else if (lineType == HORIZONTAL){
      if (yDest >= yMin[i] && yDest <= yMax[i]){
        if ((xStart >= xMin[i] && xStart <= xMax[i]) ||
          (xDest >= xMin[i] && xDest <= xMax[i]) ||
          (xStart <= xMin[i] && xDest >= xMax[i]) ||
          (xDest <= xMin[i] && xStart >= xMax[i])){
          return false;
        }
      }
    }

    else { // General line case (slope != 0 and finite)
           // Check for intersection with left side of obstacle
      double y = yStart + m*(xMin[i] - xStart);
      if (y >= yMin[i] && y <= yMax[i]){
        if ((xMin[i] >= xStart && xMin[i] <= xDest) ||
            (xMin[i] >= xDest && xMin[i] <= xStart)){
          return false;
        }
      }

      // Check for intersection with right side of obstacle
      y = yStart + m*(xMax[i] - xStart);
      if (y >= yMin[i] && y <= yMax[i]){
        if ((xMax[i] >= xStart && xMax[i] <= xDest) ||
          (xMax[i] >= xDest && xMax[i] <= xStart)){
          return false;
        }
      }

      // Check for intersection with bottom of obstacle
      double x = (yMin[i] - yStart)/m + xStart;
      if (x >= xMin[i] && x <= xMax[i]){
        if ((yMin[i] >= yStart && yMin[i] <= yDest) ||
         (yMin[i] >= yDest && yMin[i] <= yStart)){
          return false;
        }
      }

      // Check for intersection with top of obstacle
      x = (yMax[i] - yStart)/m + xStart;
      if (x >= xMin[i] && x <= xMax[i]){
        if ((yMax[i] >= yStart && yMax[i] <= yDest) ||
         (yMax[i] >= yDest && yMax[i] <= yStart)){
         return false;
        }
      }
    }
  }

  return true;
}

bool MotionPlan::equal(double x, double y){
  return (fabs(x-y) < EPSILON);
}

// First recurses through child nodes to find nearest node to xSample and ySample
// within children's subtrees, then compares the nearest node from that
// to the calling node's position itself. Returns nearest node and distance
// from nearest node to sample point.
//
MotionPlan::RRT::TreeNode* MotionPlan::RRT::TreeNode::nearestNode(double xSample,
  double ySample,
  double* nearestDist){

  (*nearestDist) = -1;
  double dist;
  double dx, dy;
  TreeNode* nearest = NULL;
  TreeNode* childNearest = NULL;

  // Find nearest node of all this node's children's trees
  std::vector<TreeNode*>::iterator child;
  for (child = children.begin(); child != children.end(); ++child){

    childNearest = (*child)->nearestNode(xSample, ySample, &dist);

    // Is this child's nearest node closer than the previous
    // nearest node?
    if (dist < (*nearestDist) || nearest == NULL){
      nearest = childNearest;
      (*nearestDist) = dist;
    }
  }

  // Compare now to this current node
  dx = x - xSample;
  dy = y - ySample;
  dist = dx*dx + dy*dy;

  if (dist < (*nearestDist) || nearest == NULL){
    nearest = this;
    (*nearestDist) = dist;
  }

  return nearest;
}

void MotionPlan::RRT::TreeNode::deleteChildren()
{
  std::vector<TreeNode*>::iterator child;
  for (child = children.begin(); child != children.end(); ++child){
    if ((*child) != NULL){
      (*child)->deleteChildren();
      delete (*child);
    }
  }

  children.clear();
}

MotionPlan::RRT::RRT(double* xMini, double* xMaxi, double* yMini, double* yMaxi, int numObs,
 double xL, double xR, double yT, double yB,
 double xS, double yS,
 double xG, double yG,
 double step) :
xMin(xMini), xMax(xMaxi), yMin(yMini), yMax(yMaxi), numObstacles(numObs),
xLeft(xL), xRight(xR), yTop(yT), yBottom(yB),
xStart(xS), yStart(yS),
xGoal(xG), yGoal(yG),
stepSize(step),
root(NULL)
{
  srand((unsigned int)time(NULL));
}

MotionPlan::RRT::RRT(std::string fileName):
root(NULL), xMin(NULL), xMax(NULL), yMin(NULL), yMax(NULL)
{
  initFromFile(fileName);
  srand((unsigned int)time(NULL));
}

// Reads initialization info for this RRT from a file with the
// following format:
// xLeft
// xRight
// yBottom
// yTop
// numObstacles
// xMin1 xMax1 yMin1 yMax1
// ...
// xMinN xMaxN yMinN yMaxN
// xStart yStart
// xGoal yGoal
// Stepsize
void MotionPlan::RRT::initFromFile(std::string fileName){
  if (xMin != NULL)
    delete [] xMin;
  if (xMax != NULL)
    delete [] xMax;
  if (yMin != NULL)
    delete [] yMin;
  if (yMax != NULL)
    delete [] yMax;

  std::ifstream input(fileName.c_str());

  input >> xLeft >> xRight >> yBottom >> yTop >> numObstacles;

  xMin = new double[numObstacles];
  xMax = new double[numObstacles];
  yMin = new double[numObstacles];
  yMax = new double[numObstacles];

  for (int i = 0; i < numObstacles; ++i){
    input >> xMin[i] >> xMax[i] >> yMin[i] >> yMax[i];
  }

  input >> xStart >> yStart >> xGoal >> yGoal >> stepSize;

  input.close();
}

MotionPlan::RRT::~RRT(){
  if (root != NULL){
    root->deleteChildren();
    delete root;
    root = NULL;
  }

  if (xMin != NULL){
    delete [] xMin;
    xMin = NULL;
  }

  if (xMax != NULL){
    delete [] xMax;
    xMax = NULL;
  }

  if (yMin != NULL){
    delete [] yMin;
    yMin = NULL;
  }

  if (yMax != NULL){
    delete [] yMax;
    yMax = NULL;
  }
}

// For x and y, generates a random number, scales it to [0,1], then
// scales it to the distance between the boundaries, then offsets it
// by the left/bottom value. This effectively generates x and y values
// in [xLeft, xRight] and [yBottom,yTop], respectively. We continue
// generating points until we find one that is clear of all obstacles.
void MotionPlan::RRT::randFreeSample(double* x, double* y){
  do{
    (*x) = (((double)rand())/RAND_MAX)*(xRight - xLeft) + xLeft;
    (*y) = (((double)rand())/RAND_MAX)*(yTop - yBottom) + yBottom;
  } while(!clear(xMin, xMax, yMin, yMax, numObstacles, *x, *y));
}

// Recurses through the RRT, calling nearestNode() on each node.
MotionPlan::RRT::TreeNode* MotionPlan::RRT::nearestNode(double x, double y){
  TreeNode* nearest = NULL;
  double distance;
  nearest = root->nearestNode(x, y, &distance);

  return nearest;
}

// Finds the vector from the nearest node to the sample point (x,y),
// normalizes it, and then scales it by stepSize to get our new point to add
// to the tree, given that link() between the nearest node and the new node
// does not fail.
MotionPlan::RRT::TreeNode* MotionPlan::RRT::genNewNode(const TreeNode* nearest,
 double x, double y){
  double dx = x - nearest->x;
  double dy = y - nearest->y;
  double dist = sqrt(dx*dx + dy*dy);

  double newX = nearest->x + stepSize*(dx / dist);
  double newY = nearest->y + stepSize*(dy / dist);

  if (link(xMin, xMax, yMin, yMax, numObstacles, nearest->x, nearest->y, newX, newY)){
    TreeNode* newNode = new TreeNode;
    newNode->x = newX;
    newNode->y = newY;

    return newNode;
  }
  else{
    return NULL;
  }
}

// Checks if the square of the distance between this node and
// the goal position is within the square of the stepSize. If so,
// check link() between the two positions. Return true if link()
// passes; false if not.
bool MotionPlan::RRT::checkGoal(const TreeNode* checkNode){
  double dx = xGoal - checkNode->x;
  double dy = yGoal - checkNode->y;

  if ((dx*dx + dy*dy) <= stepSize*stepSize)
    return link(xMin, xMax, yMin, yMax, numObstacles, checkNode->x, checkNode->y, xGoal, yGoal);
  else
    return false;
}

// First clears the tree of any info that may have been left over
// from a previous pathfind.
// While a path to the goal hasn't been found and we haven't iterated
// more than MAX_ITERATIONS:
// Generate a random sample, find the nearest node to that sample,
// attempt to make a new node stepSize away from the nearest node in the
// direction of the random sample, and if that works, add it to the tree
// and check if it's close enough to the goal to try a link() to it. If
// the goal has been reached, add a new node to the tree that represents
// the goal position.
// If the goal was added to the tree, find the path to it by starting at
// the goal and building a path backwards through its parents. Reverse this
// list, and you have the path.
// If the goal was never added to the tree, return false.
bool MotionPlan::RRT::findPath(int* iterations, int* nodePath, int* pathLength){

  if (root != NULL){
    root->deleteChildren();
  }
  else{
    root = new TreeNode;
    root->parent = NULL;
    root->nodeID = 0;
    nodes.push_back(root);
  }

  root->x = xStart;
  root->y = yStart;

  nodes.clear();
  edges.clear();

  nodes.push_back(root);

  (*iterations) = 0;
  bool goalReached = false;
  TreeNode* goal = NULL;

  while (!goalReached && (*iterations) < MAX_ITERATIONS){
    ++(*iterations);

    double sampleX, sampleY;
    TreeNode* near = NULL;
    TreeNode* newNode = NULL;

    randFreeSample(&sampleX, &sampleY);
    near = nearestNode(sampleX, sampleY);

    newNode = genNewNode(near, sampleX, sampleY);
    if (newNode != NULL){
      newNode->parent = near;
      near->children.push_back(newNode);

      newNode->nodeID = nodes.size();
      nodes.push_back(newNode);
      edges.push_back(Edge(near->nodeID, newNode->nodeID));

      if (checkGoal(newNode)){
        goal = new TreeNode;
        goal->x = xGoal;
        goal->y = yGoal;
        goal->parent = newNode;
        newNode->children.push_back(goal);
        goalReached = true;

        goal->nodeID = nodes.size();
        nodes.push_back(goal);
        edges.push_back(Edge(newNode->nodeID, goal->nodeID));
      }
    }
  } // end while goal hasn't been reached

  if (goal != NULL){

    std::vector<int> reversePath;
    for (TreeNode* current = goal; current != NULL; current = current->parent){
      reversePath.push_back(current->nodeID);
    }

    (*pathLength) = reversePath.size();

    for (int i = 0; i < (*pathLength); ++i){
      nodePath[(*pathLength)-1-i] = reversePath[i];
    }

    return true;
  }
  else{
    nodePath = NULL;
    return false;
  }
}

void MotionPlan::RRT::outputTree(std::ostream &outStream)
{
  outStream << "--- Tree Nodes ---" << std::endl;
  for (int i = 0; i < nodes.size(); ++i)
    outStream << (nodes[i])->nodeID << "\t" << (nodes[i])->x << "\t" << (nodes[i])->y << std::endl;

  outStream << "--- Tree Edges ---" << std::endl;
  for (int i = 0; i < edges.size(); ++i)
    outStream << (edges[i].node1) << "\t" << (edges[i].node2) << std::endl;

  // outStream << "--- Node1 (x,y), Node2 (x,y)  ---" << std::endl;
  // for (int i = 0; i < edges.size(); ++i){
  //   outStream << (nodes[edges[i].node1])->x << "\t" << (nodes[edges[i].node1])->y << std::endl;
  //   outStream << (nodes[edges[i].node2])->x << "\t" << (nodes[edges[i].node2])->y << std::endl;
  //   outStream << "\n" << std::endl;
  //}
}

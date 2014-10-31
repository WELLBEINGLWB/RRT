#include "MotionPlan.h"

// (xTest, yTest)が障害物の中にあるかどうかの判定
bool MotionPlan::clear(const double* xMin, const double* xMax,
                       const double* yMin, const double* yMax,
                       const double* zMin, const double* zMax,
                       int numObstacles,
                       double xTest, double yTest, double zTest)
{
  for (int i = 0; i < numObstacles; ++i) { // 障害物の範囲内ならreturn false
    if (xMin[i] <= xTest && xTest <= xMax[i] &&
        yMin[i] <= yTest && yTest <= yMax[i] &&
        zMin[i] <= zTest && zTest <= zMax[i]) {
      return false;
    }
  }

  return true; // すべての障害物の中に入ってなかったらreturn true
}



void MotionPlan::CreateGridPoint(const double* xMin, const double* xMax,
                                 const double* yMin, const double* yMax,
                                 const double* zMin, const double* zMax, POINT *P, int i)
{
  P[0].x = xMin[i]; P[0].y = yMin[i]; P[0].z = zMin[i]; // x,y,zの最小値、最大値からそれぞれの座標を割り出し、Pに代入。
  P[1].x = xMax[i]; P[1].y = yMin[i]; P[1].z = zMin[i];
  P[2].x = xMax[i]; P[2].y = yMax[i]; P[2].z = zMin[i];
  P[3].x = xMin[i]; P[3].y = yMax[i]; P[3].z = zMin[i];
  P[4].x = xMin[i]; P[4].y = yMin[i]; P[4].z = zMax[i];
  P[5].x = xMax[i]; P[5].y = yMin[i]; P[5].z = zMax[i];
  P[6].x = xMax[i]; P[6].y = yMax[i]; P[6].z = zMax[i];
  P[7].x = xMin[i]; P[7].y = yMax[i]; P[7].z = zMax[i];
}



void MotionPlan::PlaneEquation(POINT p[], int i0[], int i1[], int i2[], int i, double a[])
{
  // 平面の方程式の係数を導出 (ax+by+cz=d)
  // http://keisan.casio.jp/has10/SpecExec.cgi
  a[0] = (p[i1[i]].y-p[i0[i]].y)*(p[i2[i]].z-p[i0[i]].z)-(p[i2[i]].y-p[i0[i]].y)*(p[i1[i]].z-p[i0[i]].z);
  a[1] = (p[i1[i]].z-p[i0[i]].z)*(p[i2[i]].x-p[i0[i]].x)-(p[i2[i]].z-p[i0[i]].z)*(p[i1[i]].x-p[i0[i]].x);
  a[2] = (p[i1[i]].x-p[i0[i]].x)*(p[i2[i]].y-p[i0[i]].y)-(p[i2[i]].x-p[i0[i]].x)*(p[i1[i]].y-p[i0[i]].y);
  a[3] = a[0]*p[i0[i]].x + a[1]*p[i0[i]].y + a[2]*p[i0[i]].z;

  //cout << a[0] << " x + " << a[1] << " y + " << a[2] <<  " z + " << a[3] << " = 0" << endl;
}



void MotionPlan::Pcompare(POINT A, POINT B, POINT *compare)
{
  // 与えられた2点のどちらが小さいか、配列compareに小さい順に格納
  if ((A.x - B.x) > 0) {
    compare[0].x = B.x;
    compare[1].x = A.x;
  }else{
    compare[0].x = A.x;
    compare[1].x = B.x;
  }

  if((A.y - B.y) > 0) {
    compare[0].y = B.y;
    compare[1].y = A.y;
  }else{
    compare[0].y = A.y;
    compare[1].y = B.y;
  }

  if((A.z - B.z) > 0) {
    compare[0].z = B.z;
    compare[1].z = A.z;
  }else{
    compare[0].z = A.z;
    compare[1].z = B.z;
  }
}



bool MotionPlan::link(const double* xMin, const double* xMax,
                      const double* yMin, const double* yMax,
                      const double* zMin, const double* zMax,
                      int numObstacles,
                      double xStart, double yStart, double zStart,
                      double xDest, double yDest, double zDest)
{
  int i = 0;
  bool flag = true;
  POINT A, B, P, p[8], E, compare[2];
  double a[4];
  double t, DE;
  int i0[6] = {0,0,1,2,3,4}; // 配列の組み合わせはノート見る。
  int i1[6] = {1,1,2,3,4,5}; // 0から5までの6面定義、3点で考える。
  int i2[6] = {2,4,5,6,7,6};

  A.x = xStart; A.y = yStart; A.z = zStart;
  B.x = xDest ; B.y = yDest ; B.z = zDest;
  Pcompare(A, B, compare);

  // 平面と直線の交点を導出 (参考サイト：http://www.hiramine.com/programming/graphics/3d_planesegmentintersection.html)
  E.x = A.x - B.x;
  E.y = A.y - B.y;
  E.z = A.z - B.z;

  for (int j = 0; j < numObstacles; ++j) {
    CreateGridPoint(xMin, xMax, yMin, yMax, zMin, zMax, p, j);

    do{
      PlaneEquation(p, i0, i1, i2, i, a);

      t = (a[3] - (a[0] * A.x + a[1] * A.y + a[2] * A.z)) / (a[0] * E.x + a[1] * E.y + a[2] * E.z);
      DE = a[0] * E.x + a[1] * E.y + a[2] * E.z;

      if(DE == 0){
        // std::cout << j+1 << "番目の障害物: " << i << "面と直線との交点なし" << std::endl;
      }else{
        // std::cout << "交点あり" << endl;
        P.x = A.x + t * E.x;
        P.y = A.y + t * E.y;
        P.z = A.z + t * E.z;
        // std::cout << j+1 << "番目の障害物: " << i << "面, (" << P.x << ", " << P.y << ", " << P.z << ")" << std::endl;

        if (xMin[j] <= P.x && P.x <= xMax[j] &&
            yMin[j] <= P.y && P.y <= yMax[j] &&
            zMin[j] <= P.z && P.z <= zMax[j] &&
            compare[0].x <= P.x && P.x <= compare[1].x &&
            compare[0].y <= P.y && P.y <= compare[1].y &&
            compare[0].z <= P.z && P.z <= compare[1].z) {
          // std::cout << j+1 << "番目の障害物: " << i << "面, 定義した4点内に入っている" << std::endl;
          flag = false;
          break;
        }else{
          // std::cout << j+1 << "番目の障害物: " << i << "面, 範囲外" << std::endl;
        }
      }
      i++;
    }while(i<6);
    i = 0;
  }

  return flag;
}



// childrenのsubtreesの中で(xSample, ySample, zSample)に一番近いノードを探す(再帰的に)
// sample点から一番近いノードと距離をリターン
MotionPlan::RRT::TreeNode* MotionPlan::RRT::TreeNode::nearestNode(double xSample, double ySample, double zSample, double* nearestDist)
{
  (*nearestDist) = -1;
  double dist;
  double dx, dy, dz;
  TreeNode* nearest = NULL;
  TreeNode* childNearest = NULL;

  // すべてのchildren's treesのノードのなかで最も近いものを探す！
  std::vector<TreeNode*>::iterator child;
  for (child = children.begin(); child != children.end(); ++child) {
    childNearest = (*child)->nearestNode(xSample, ySample, zSample, &dist);

    // 今のやつが以前の最も近かったノードよりも近かったら
    if (dist < (*nearestDist) || nearest == NULL){
      nearest = childNearest;
      (*nearestDist) = dist;
    }
  }

  //いまのノードと比較
  dx = x - xSample;
  dy = y - ySample;
  dz = z - zSample;
  dist = dx*dx + dy*dy + dz*dz;

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



MotionPlan::RRT::RRT(double* xMini, double* xMaxi, double* yMini, double* yMaxi, double* zMini, double* zMaxi,int numObs,
                     double xL, double xR, double yT, double yB, double zT, double zB,
                     double xS, double yS, double zS,
                     double xG, double yG, double zG,
                     double step) :
                     root(NULL),
                     xMin(xMini), xMax(xMaxi), yMin(yMini), yMax(yMaxi), zMin(zMini), zMax(zMaxi),
                     numObstacles(numObs),
                     xStart(xS), yStart(yS), zStart(zS),
                     xGoal(xG), yGoal(yG), zGoal(zG),
                     stepSize(step),
                     xLeft(xL), xRight(xR), yTop(yT), yBottom(yB), zTop(zT), zBottom(zB)
{
  srand((unsigned int)time(NULL));
}



MotionPlan::RRT::RRT(std::string fileName):
                     root(NULL),
                     xMin(NULL), xMax(NULL),
                     yMin(NULL), yMax(NULL),
                     zMin(NULL), zMax(NULL)
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
void MotionPlan::RRT::initFromFile(std::string fileName)
{
  if (xMin != NULL)
    delete [] xMin;
  if (xMax != NULL)
    delete [] xMax;
  if (yMin != NULL)
    delete [] yMin;
  if (yMax != NULL)
    delete [] yMax;
  if (zMin != NULL)
    delete [] zMin;
  if (zMax != NULL)
    delete [] zMax;

  std::ifstream input(fileName.c_str());

  input >> xLeft >> xRight >> yBottom >> yTop >> zBottom >> zTop >> numObstacles;

  xMin = new double[numObstacles];
  xMax = new double[numObstacles];
  yMin = new double[numObstacles];
  yMax = new double[numObstacles];
  zMin = new double[numObstacles];
  zMax = new double[numObstacles];

  for (int i = 0; i < numObstacles; ++i){
    input >> xMin[i] >> xMax[i] >> yMin[i] >> yMax[i] >> zMin[i] >> zMax[i];
  }

  input >> xStart >> yStart >> zStart >> xGoal >> yGoal >> zGoal >> stepSize;

  input.close();
}



MotionPlan::RRT::~RRT()
{
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

  if (zMin != NULL){
    delete [] zMin;
    zMin = NULL;
  }

  if (zMax != NULL){
    delete [] zMax;
    zMax = NULL;
  }
}



// For x and y, generates a random number, scales it to [0,1], then
// scales it to the distance between the boundaries, then offsets it
// by the left/bottom value. This effectively generates x and y values
// in [xLeft, xRight] and [yBottom, yTop], respectively. We continue
// generating points until we find one that is clear of all obstacles.
void MotionPlan::RRT::randFreeSample(double* x, double* y, double* z)
{
  do{
    (*x) = (((double)rand())/RAND_MAX)*(xRight - xLeft) + xLeft;
    (*y) = (((double)rand())/RAND_MAX)*(yTop - yBottom) + yBottom;
    (*z) = (((double)rand())/RAND_MAX)*(zTop - zBottom) + zBottom;
  } while(!clear(xMin, xMax, yMin, yMax, zMin, zMax, numObstacles, *x, *y, *z));
}



// Recurses through the RRT, calling nearestNode() on each node.
MotionPlan::RRT::TreeNode* MotionPlan::RRT::nearestNode(double x, double y, double z)
{
  TreeNode* nearest = NULL;
  double distance;

  nearest = root->nearestNode(x, y, z, &distance);

  return nearest;
}



// Finds the vector from the nearest node to the sample point (x,y),
// normalizes it, and then scales it by stepSize to get our new point to add
// to the tree, given that link() between the nearest node and the new node
// does not fail.
MotionPlan::RRT::TreeNode* MotionPlan::RRT::genNewNode(const TreeNode* nearest, double x, double y, double z)
{
  double dx = x - nearest->x;
  double dy = y - nearest->y;
  double dz = z - nearest->z;
  double dist = sqrt(dx * dx + dy * dy + dz * dz);

  double newX = nearest->x + stepSize * (dx / dist);
  double newY = nearest->y + stepSize * (dy / dist);
  double newZ = nearest->z + stepSize * (dz / dist);

  if (link(xMin, xMax, yMin, yMax, zMin, zMax,
           numObstacles,
           nearest->x, nearest->y, nearest->z, newX, newY, newZ)){

    TreeNode* newNode = new TreeNode;
    newNode->x = newX;
    newNode->y = newY;
    newNode->z = newZ;

    return newNode;
  } else{
    return NULL;
  }
}



// Checks if the square of the distance between this node and
// the goal position is within the square of the stepSize. If so,
// check link() between the two positions. Return true if link()
// passes; false if not.
bool MotionPlan::RRT::checkGoal(const TreeNode* checkNode)
{
  double dx = xGoal - checkNode->x;
  double dy = yGoal - checkNode->y;
  double dz = zGoal - checkNode->z;

  if ((dx*dx + dy*dy + dz*dz) <= stepSize*stepSize){
    return link(xMin, xMax, yMin, yMax, zMin, zMax, numObstacles, checkNode->x, checkNode->y, checkNode->z, xGoal, yGoal, zGoal);
  } else{
    return false;
  }
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

bool MotionPlan::RRT::findPath(int* iterations, int* nodePath, int* pathLength)
{

  if (root != NULL){
    root->deleteChildren();
  } else{
    root = new TreeNode;
    root->parent = NULL;
    root->nodeID = 0;
    nodes.push_back(root);
  }

  root->x = xStart;
  root->y = yStart;
  root->z = zStart;

  nodes.clear();
  edges.clear();

  nodes.push_back(root);

  (*iterations) = 0;
  bool goalReached = false;
  TreeNode* goal = NULL;

  while (!goalReached && (*iterations) < MAX_ITERATIONS){
    ++(*iterations);

    if ((*iterations)%1000 == 0){
      std::cout << "iterations = " << (*iterations) << std::endl;
    }

    double sampleX, sampleY, sampleZ;
    TreeNode* near = NULL;
    TreeNode* newNode = NULL;

    randFreeSample(&sampleX, &sampleY, &sampleZ);
    std::cout << "(sampleX, sampleY, sampleZ) = (" << sampleX << ", " << sampleY << ", " << sampleZ  << " )"<< std::endl;

    near = nearestNode(sampleX, sampleY, sampleZ);

    newNode = genNewNode(near, sampleX, sampleY, sampleZ);
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
        goal->z = zGoal;
        goal->parent = newNode;
        newNode->children.push_back(goal);
        goalReached = true;

        goal->nodeID = nodes.size();
        nodes.push_back(goal);
        edges.push_back(Edge(newNode->nodeID, goal->nodeID));
      }
    }

    // while(1){
    //   std::string a;
    //   std::cout << (*iterations) << "ループに進む場合は「y」を入力"<< std::endl;
    //   std::cin >> a;
    //   if( a == "y" ) break;
    // }


  } // end while goal hasn't been reached

  std::cout << "iterations = " << (*iterations) << " Finished" << std::endl;

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
  } else {
    nodePath = NULL;
    return false;
  }
}



void MotionPlan::RRT::CreateCube(std::ostream &cube)
{
  RANGE obstacle;
  std::ofstream plot("./plot_data/start_goal.dat");

  for(int ob = 0; ob < numObstacles; ++ob){
    obstacle.xrange[0] = xMin[ob]; obstacle.yrange[0] = yMin[ob]; obstacle.zrange[0] = zMin[ob];
    obstacle.xrange[1] = xMax[ob]; obstacle.yrange[1] = yMax[ob]; obstacle.zrange[1] = zMax[ob];
    for (int i = 0; i < 2; ++i){
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << std::endl;
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << std::endl;
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[1] << "\t" << obstacle.zrange[i] << std::endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[1] << "\t" << obstacle.zrange[i] << std::endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << std::endl;
      cube << "\n\n";
    }

    for (int i = 0; i < 2; ++i){
      for (int j = 0; j < 2; ++j){
        for (int k = 0; k < 2; ++k){
          cube << obstacle.xrange[i] << "\t" << obstacle.yrange[j] << "\t" << obstacle.zrange[k] << std::endl;
        }
        cube << "\n\n";
      }
    }
  }
  plot << xStart << "\t" << yStart << "\t" << zStart << std::endl;
  plot << xGoal << "\t" << yGoal << "\t" << zGoal << std::endl;

}



void MotionPlan::RRT::outputTree(std::ostream &outStream)
{
  //ノードの座標を2点ずつのブロックでファイルに書き込み
  for (int i = 0; i < edges.size(); ++i){
    outStream << (nodes[edges[i].node1])->x << "\t" << (nodes[edges[i].node1])->y << "\t" << (nodes[edges[i].node1])->z << std::endl;
    outStream << (nodes[edges[i].node2])->x << "\t" << (nodes[edges[i].node2])->y << "\t" << (nodes[edges[i].node2])->z << std::endl;
    outStream << "\n" << std::endl;

    //usleep(5000);
  }
}



void MotionPlan::RRT::outputTree(FILE *outStream)
{
  for (int i = 0; i < edges.size(); ++i){
    std::cout << i << "番目のループだよ" << std::endl;
    //fprintf(outStream, "plot '-' using 1:2 with lines");

    fprintf(outStream,"%lf %lf %lf\n", ((nodes[edges[i].node1])->x), ((nodes[edges[i].node1])->y), ((nodes[edges[i].node1])->z));
    fprintf(outStream,"%lf %lf %lf\n\n", ((nodes[edges[i].node2])->x), ((nodes[edges[i].node2])->y), ((nodes[edges[i].node2])->z));
    fprintf(outStream,"e\n"); //データ書き込み終了を知らせる．
    //}
    fflush(outStream);


    for(int k=0; k<50; ++k){
      usleep(10000);
    }

  }
}



void MotionPlan::RRT::gnuplot_config(FILE *outStream)
{

  outStream = popen("gnuplot -persist", "w");    /*プログラムが終了後GnuplotのWindowを閉じない*/
  //outStream = popen("gnuplot", "w");               /*プログラムが終了後GnuplotのWindowを閉じる*/
  fprintf(outStream,"set terminal gif animate optimize size \n");
  fprintf(outStream,"set output 'anime.gif'\n");

  /*アニメーションをgif動画として保存する場合は，上のコメントを解除*/
  fprintf(outStream,"set xtics 0.5\n");
  fprintf(outStream,"set ytics 0.5\n");
  fprintf(outStream,"set xlabel 'x' font 'Times, 20'\n");
  fprintf(outStream,"set ylabel 'y' font 'Times, 20'\n");
  fprintf(outStream,"set grid\n");
  fprintf(outStream,"set size square\n");
}
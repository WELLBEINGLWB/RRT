#include "MotionPlan.h"

// (xTest, yTest)が障害物の中にあるかどうかの判定
bool MotionPlan::clear(const double* xMin, const double* xMax,
                       const double* yMin, const double* yMax, int numObstacles,
                       double xTest, double yTest)
{
  for (int i = 0; i < numObstacles; ++i) { // 障害物の範囲内ならreturn false
    if (xTest >= xMin[i] && xTest <= xMax[i] && yTest >= yMin[i] &&
        yTest <= yMax[i]) {
      return false;
    }
  }

  return true; // すべての障害物の中に入ってなかったらreturn true
}


// 最初に(xStart,yStart)と(xDest,yDest)の間の線を見つける。'link-line'と呼ぶよ。
// それぞれの障害物において、'link-line'が0、無限か有限なのかどうかの依存をチェック。

// VERTICAL/HORIZONTAL
// 水平/垂直
// link-lineの終端のどちらかが障害物か衝突の中にある場合、
// 水平の場合、link-lineはx = aで、もしxが障害物のxMinとxMaxの範囲内やまたがっていたら...
// 垂直の場合、link-lineはy = aで、.....垂直と一緒
// その他の場合なら障害物とあたってない！

// NONZERO SLOPE
// 障害物のそれぞれのサイドにおいて、(y-y1)=m(x-x1)で考える。
bool MotionPlan::link(const double* xMin, const double* xMax,
                      const double* yMin, const double* yMax, int numObstacles,
                      double xStart, double yStart, double xDest, double yDest)
{
  if (!clear(xMin, xMax, yMin, yMax, numObstacles, xStart, yStart) ||
      !clear(xMin, xMax, yMin, yMax, numObstacles, xDest, yDest)) {
    return false;
  }

  double dx = xDest - xStart;
  double dy = yDest - yStart;
  double m = 0.0;
  LineType lineType = GENERAL;

  if (MotionPlan::equal(dx, 0))
    lineType = VERTICAL;
  else if (MotionPlan::equal(dy, 0))
    lineType = HORIZONTAL;
  else
    m = dy / dx;

  for (int i = 0; i < numObstacles; ++i) {
    if (lineType == VERTICAL) {
      if (xDest >= xMin[i] && xDest <= xMax[i]) {
        if ((yStart >= yMin[i] && yStart <= yMax[i]) ||
            (yDest >= yMin[i] && yDest <= yMax[i]) ||
            (yStart <= yMin[i] && yDest >= yMax[i]) ||
            (yDest <= yMin[i] && yStart >= yMax[i])) {
          return false;
        }
      }
    }

    else if (lineType == HORIZONTAL) {
      if (yDest >= yMin[i] && yDest <= yMax[i]) {
        if ((xStart >= xMin[i] && xStart <= xMax[i]) ||
            (xDest >= xMin[i] && xDest <= xMax[i]) ||
            (xStart <= xMin[i] && xDest >= xMax[i]) ||
            (xDest <= xMin[i] && xStart >= xMax[i])) {
          return false;
        }
      }
    }

    else {  // General line case (slope != 0 and finite)
            // Check for intersection with left side of obstacle
      double y = yStart + m * (xMin[i] - xStart);
      if (y >= yMin[i] && y <= yMax[i]) {
        if ((xMin[i] >= xStart && xMin[i] <= xDest) ||
            (xMin[i] >= xDest && xMin[i] <= xStart)) {
          return false;
        }
      }

      // Check for intersection with right side of obstacle
      y = yStart + m * (xMax[i] - xStart);
      if (y >= yMin[i] && y <= yMax[i]) {
        if ((xMax[i] >= xStart && xMax[i] <= xDest) ||
            (xMax[i] >= xDest && xMax[i] <= xStart)) {
          return false;
        }
      }

      // Check for intersection with bottom of obstacle
      double x = (yMin[i] - yStart) / m + xStart;
      if (x >= xMin[i] && x <= xMax[i]) {
        if ((yMin[i] >= yStart && yMin[i] <= yDest) ||
            (yMin[i] >= yDest && yMin[i] <= yStart)) {
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


void MotionPlan::MoveObstacle(double* xMin, double* xMax, double* yMin, double* yMax, int numObstacles)
{
  // for (int i = 0; i < numObstacles; ++i) { // 障害物の範囲内ならreturn false
  //   if(i == 0){
  //     yMin[i] = yMin[i] + 0.1*iterations;
  //     yMax[i] = yMax[i] + 0.1*iterations;
  //   }else if(i == 1){
  //     yMin[i] = yMin[i] - 0.1*iterations;
  //     yMax[i] = yMax[i] - 0.1*iterations;
  //   }else{
  //     //そのまま
  //   }
  //   std::cout << "(yMin[i], yMax[i]) = (" << yMin[i] << "," << yMax[i] << ")" << std::endl;
  // }

  std::ofstream cube("./plot_data/testcase1_obstacle.dat", std::ios_base::trunc);
  RANGE obstacle;
  for(int ob = 0; ob < numObstacles; ++ob){
    obstacle.xrange[0] = xMin[ob]; obstacle.yrange[0] = yMin[ob];
    obstacle.xrange[1] = xMax[ob]; obstacle.yrange[1] = yMax[ob];
    for (int i = 0; i < 2; ++i){
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << std::endl;
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[0] << "\t" << std::endl;
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[1] << "\t" << std::endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[1] << "\t" << std::endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << std::endl;
      cube << "\n\n";
    }
  }
}


bool MotionPlan::equal(double x, double y) { return (fabs(x - y) < EPSILON); }


// First recurses through child nodes to find nearest node to xSample and
// ySample
// within children's subtrees, then compares the nearest node from that
// to the calling node's position itself. Returns nearest node and distance
// from nearest node to sample point.
MotionPlan::RRT::TreeNode* MotionPlan::RRT::TreeNode::nearestNode(double xSample, double ySample, double* nearestDist)
{
  (*nearestDist) = -1;
  double dist;
  double dx, dy;
  TreeNode* nearest = NULL;
  TreeNode* childNearest = NULL;

  // Find nearest node of all this node's children's trees
  std::vector<TreeNode*>::iterator child;
  for (child = children.begin(); child != children.end(); ++child) {
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
 root(NULL),
 xMin(xMini), xMax(xMaxi), yMin(yMini), yMax(yMaxi),
 numObstacles(numObs),
 xStart(xS), yStart(yS),
 xGoal(xG), yGoal(yG),
 stepSize(step),
 xLeft(xL), xRight(xR), yTop(yT), yBottom(yB)
{
  srand((unsigned int)time(NULL));
}


MotionPlan::RRT::RRT(std::string fileName):
 root(NULL), xMin(NULL), xMax(NULL), yMin(NULL), yMax(NULL)
{
  initFromFile(fileName);
  MoveObstacle(xMin, xMax, yMin, yMax, numObstacles);
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

  std::ofstream Start_and_Goal("./plot_data/start_goal.dat");

  Start_and_Goal << xStart << "\t" << yStart << std::endl;
  Start_and_Goal << xGoal << "\t" << yGoal << std::endl;

  std::ofstream pathinit("./plot_data/path_data.dat", std::ios_base::trunc);
  pathinit << xStart << "\t" << yStart << std::endl;
  pathinit << xStart << "\t" << yStart << std::endl;

  std::ofstream smoothData("./plot_data/path_data_mod.dat", std::ios_base::trunc);
  smoothData << xStart << "\t" << yStart << std::endl;
  smoothData << xStart << "\t" << yStart << std::endl;
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
    delete[] yMin;
    yMin = NULL;
  }

  if (yMax != NULL) {
    delete[] yMax;
    yMax = NULL;
  }
}


// For x and y, generates a random number, scales it to [0,1], then
// scales it to the distance between the boundaries, then offsets it
// by the left/bottom value. This effectively generates x and y values
// in [xLeft, xRight] and [yBottom,yTop], respectively. We continue
// generating points until we find one that is clear of all obstacles.
void MotionPlan::RRT::randFreeSample(double* x, double* y)
{
  int roulette;

  //srand((unsigned int)time(NULL));
  roulette = (((double)rand())/RAND_MAX)*100;
  //std::cout << "ルーレットは" << roulette << "です！" << std::endl;
  if(roulette >= 90){
    (*x) = xGoal;
    (*y) = yGoal;
  }else{
    do{
      (*x) = (((double)rand())/RAND_MAX)*(xRight - xLeft) + xLeft;
      (*y) = (((double)rand())/RAND_MAX)*(yTop - yBottom) + yBottom;
    } while(!clear(xMin, xMax, yMin, yMax, numObstacles, *x, *y));
  }

}


// Recurses through the RRT, calling nearestNode() on each node.
MotionPlan::RRT::TreeNode* MotionPlan::RRT::nearestNode(double x, double y)
{
  TreeNode* nearest = NULL;
  double distance;
  nearest = root->nearestNode(x, y, &distance);

  return nearest;
}


// Finds the vector from the nearest node to the sample point (x,y),
// normalizes it, and then scales it by stepSize to get our new point to add
// to the tree, given that link() between the nearest node and the new node
// does not fail.
MotionPlan::RRT::TreeNode* MotionPlan::RRT::genNewNode(const TreeNode* nearest, double x, double y)
{ // x, yはサンプルポイント
  double dx = x - nearest->x;
  double dy = y - nearest->y;
  double dist;
  double newdist; // 新しくできたノードを結んだ距離計算用

  dist = sqrt(dx*dx + dy*dy);
  double newX = nearest->x + stepSize*(dx / dist);
  double newY = nearest->y + stepSize*(dy / dist);


  newdist = sqrt((newX-nearest->x)*(newX-nearest->x) + (newY-nearest->y)*(newY-nearest->y));
  if(newdist > dist){
    //std::cout << "サンプルポイントをnewNodeにしたよ。座標(" << x << ", " << y << ")" << std::endl;
    newX = x;
    newY = y;
  }

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


bool MotionPlan::RRT::PathCheck(int* pathLength){
  bool check = true;

  for (int i = 0; i < (*pathLength)-1; ++i){
    if(link(xMin, xMax, yMin, yMax, numObstacles, paths[i].x, paths[i].y, paths[i+1].x, paths[i+1].y)){
      //問題なし
      check = true;
    }else{
      std::cout << "以前の作成した軌道じゃダメ。再計算" << std::endl;
      xStart = paths[i-2].x;
      yStart = paths[i-2].y;
      paths.erase(paths.begin()+i-2, paths.end());
      check = false;
      break;
    }
  }

  if(check){
    std::cout << "さっきの軌道でぜーんぜん問題なし！" << std::endl;
    return true;
  }else{
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
  int i = 0;
  // int j = 0;
  double sampleX = 0.0, sampleY = 0.0;

  // std::ofstream real("./plot_data/realtimedata.dat");
  // std::string SamplePointNow = "./plot_data/samplenow.dat";
  // std::string SamplePointNext = "./plot_data/samplenext.dat";

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

  nodes.clear();
  edges.clear();

  nodes.push_back(root);

  (*iterations) = 0;
  bool goalReached = false;
  TreeNode* goal = NULL;

  while (!goalReached && (*iterations) < MAX_ITERATIONS){
    ++(*iterations);
    i = (*iterations);

    if ((*iterations)%1000 == 0){
      std::cout << "iterations = " << (*iterations) << std::endl;
    }

    TreeNode* near = NULL;
    TreeNode* newNode = NULL;

    // outputSample(SamplePointNow, sampleX, sampleY);
    randFreeSample(&sampleX, &sampleY);
    // outputSample(SamplePointNext, sampleX, sampleY);


    near = nearestNode(sampleX, sampleY);
    newNode = genNewNode(near, sampleX, sampleY);

    if (newNode != NULL){
      newNode->parent = near;
      near->children.push_back(newNode);

      newNode->nodeID = nodes.size();
      nodes.push_back(newNode);
      edges.push_back(Edge(near->nodeID, newNode->nodeID));

      // std::cout << "(i, sampleX, sampleY) = ( " << i << ", " << sampleX << ", " << sampleY << " )" << std::endl;
      // printf("(i, sampleX, sampleY) = (%d, %2.5lf, %2.5lf )\n", i, sampleX, sampleY);
      // std::cout << (nodes[edges[j].node1])->x << "\t" << (nodes[edges[j].node1])->y << std::endl;
      // std::cout << (nodes[edges[j].node2])->x << "\t" << (nodes[edges[j].node2])->y << std::endl;

      // outputTree(real, j);
      // outputTree(j); //そのループで生成した枝をプロットする用データを生成(2つ改行で連続データ)ように書いてました！
      //j++;

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
        // outputTree(real, j);
        // outputTree(j);
      }
    }

  } // ゴールに到達するまでループ

  //連打する用窓破壊！！
  //cvReleaseImage(&img);
  //cvDestroyWindow("連打用窓");

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


void MotionPlan::RRT::RRTloop(int* iterations, int* nodePath, int* pathLength, std::ostream& nodeData)
{
  std::string a;
  POINT tmp;

  while(1){
    if (findPath(iterations, nodePath, pathLength)) {
      outputTree(nodeData);

      for (int j = 0; j < (*pathLength); ++j) {
        tmp.x = (nodes[nodePath[j]])->x;
        tmp.y = (nodes[nodePath[j]])->y;
        paths.push_back(tmp);
      }

      std::ofstream pathData("./plot_data/path_data.dat", std::ios_base::trunc);
      for(int addpath = 0; addpath < paths.size(); addpath++ ){
        pathData << paths[addpath].x << "\t" << paths[addpath].y << std::endl;
      }
      break;

    } else {
      std::cout << "Path not found." << std::endl;
    }
  }
  smoothing(100000);

}


int MotionPlan::RRT::GetRandom(double min, double max)
{
  int R;

  R = min + (int)(rand() * ((max - min) + 1.0) / (1.0 + RAND_MAX));
  return R;
}


double MotionPlan::RRT::Distance()
{
  double dis = 0.0;
  if(paths.size() == 0){
    std::cout << "パスのデータが存在しません。" << std::endl;
    return -1;
  }else{
    for (unsigned int i = 0; i < paths.size()-1; ++i) {
      dis += sqrt(pow(paths[i+1].x - paths[i].x, 2) + pow(paths[i+1].y - paths[i].y, 2));
    }
    // cout << "パスの距離は" << dis << "です。" << endl;
    return dis;
  }
}


void MotionPlan::RRT::smoothing(int loop)
{
  int SamplePoint[2];
  int tmp;
  double ini;
  double current, old = 0.0;
  int count = 0;

  ini = Distance();
  std::cout << "初期のパスの距離は" << ini << std::endl;

  for (int i = 0; i < loop; ++i){
    while (1) {
      for (int j = 0; j < 2; ++j) {
        SamplePoint[j] = GetRandom(0, paths.size() - 1);
      }
      if (SamplePoint[0] > SamplePoint[1] && SamplePoint[0] != SamplePoint[1] + 1) {
        tmp = SamplePoint[0];
        SamplePoint[0] = SamplePoint[1];
        SamplePoint[1] = tmp;
        break;
      } else if (SamplePoint[0] < SamplePoint[1] && SamplePoint[0] + 1 != SamplePoint[1]) {
        break;
      } else {
        //もう一回引き直し(ΦωΦ)
      }
    }

    if (link(xMin, xMax, yMin, yMax, numObstacles,
             paths[SamplePoint[0]].x, paths[SamplePoint[0]].y,
             paths[SamplePoint[1]].x, paths[SamplePoint[1]].y)){
      std::cout << SamplePoint[0] << "と" << SamplePoint[1] << "の間の点はグッバイ！" << std::endl;
      std::cout << i+1 << "ループ目、グッバイしたあとのpathの長さは" << paths.size() << "です。" << std::endl;
      paths.erase(paths.begin()+SamplePoint[0]+1, paths.begin()+SamplePoint[1]);

      std::ofstream outStream("./plot_data/path_data_mod.dat", std::ios_base::trunc);
      for (unsigned int k = 0; k < paths.size(); k++) {
        outStream << paths[k].x << "\t" << paths[k].y << std::endl;
      }
      for (int k = 0; k < 100; ++k){
        usleep(10000);
      }

      current = Distance();
      std::cout << "現在のパスの総距離は" << current << std::endl;
      // if( (ini - current) > 7){
      //   cout << "しきい値以下になりました！" << endl;
      //   break;
      // }
      if( fabs(old - current) < 0.001 ){
        std::cout << count + 1 << "回目、しきい値以下になりました！" << std::endl;
        count++;
      }
      old = current;

      if(count >= 5){
        std::cout << "我慢ならん！ブレイクだ！！" << std::endl;
        break;
      }

    }else{
      std::cout << i+1 << "ループ目はグッバイできませんでした。" << endl;
    }

  }
  //PrintData();
  printf("グッバイする前のパスの総距離は%5.3lf\n", ini);
  printf("        した後のパスの総距離は%5.3lf\n", Distance());
}


void MotionPlan::RRT::outputTree(std::ostream &outStream)
{
  // ノードの座標を2点ずつのブロックでファイルに書き込み
  for (int i = 0; i < edges.size(); ++i){
    outStream << (nodes[edges[i].node1])->x << "\t" << (nodes[edges[i].node1])->y << std::endl;
    outStream << (nodes[edges[i].node2])->x << "\t" << (nodes[edges[i].node2])->y << std::endl;
    outStream << "\n" << std::endl;

    for (int k = 0; k < 100; ++k){
      usleep(100);
    }

  }
}


void MotionPlan::RRT::outputTree(std::ostream &outStream, int iterations)
{
  // そのループで生成した枝をプロットする用データを生成(2つ改行で連続データ)
  outStream << (nodes[edges[iterations].node1])->x << "\t" << (nodes[edges[iterations].node1])->y << std::endl;
  outStream << (nodes[edges[iterations].node2])->x << "\t" << (nodes[edges[iterations].node2])->y << std::endl;
  outStream << "\n" << std::endl;
}


void MotionPlan::RRT::outputTree(int iterations)
{
  // 新たに生成された枝をプロットする用のデータ作成(この関数が呼び出されるたびに消去して上書き)
  std::ofstream outStream("./plot_data/single.dat", std::ios_base::trunc);
  outStream << (nodes[edges[iterations].node1])->x << "\t" << (nodes[edges[iterations].node1])->y << std::endl;
  outStream << (nodes[edges[iterations].node2])->x << "\t" << (nodes[edges[iterations].node2])->y << std::endl;
}


// void MotionPlan::RRT::outputSample(std::string Filename, double x, double y)
// {
//   // サンプリングポイントをプロットする用
//   std::ofstream real(Filename, std::ios_base::trunc);
//   real << x << "\t" << y << std::endl;
// }
#include "MotionPlan.h"

/// Simple iterates over each object in the space
/// and checks whether the test point lies inside the obstacle.
bool MotionPlan::clear(double xTest, double yTest, std::vector<POINT> &vobstacle){
  if(f_xy(xTest, yTest, vobstacle) > Threshold){
    return false;
  }else{
    return true;
  }
}


bool MotionPlan::link(double xStart, double yStart,
                      double xDest, double yDest,
                      std::vector<POINT> &vobstacle, double stepSize)
{
  double dx = xDest - xStart;
  double dy = yDest - yStart;
  double dist = sqrt(dx * dx + dy * dy);

  double CheckX;
  double CheckY;

  double Potential;
  double MaxPotential = 0.0;

  // POINT MaxPoint;
  // bool flag = true;

  // std::ofstream outStream("./plot_data/PotentialData.dat");

  if (!clear(xStart, yStart, vobstacle) ||
      !clear(xDest, yDest, vobstacle)) {
    //std::cout << "スタートとゴールが障害物内" << std::endl;
    return false;
  }

  for(double length = 0.0; length < dist; length += 0.5*stepSize){
    CheckX = xStart + length * (dx / dist);
    CheckY = yStart + length * (dy / dist);
    Potential = f_xy(CheckX, CheckY, vobstacle);
    // outStream << CheckX << "\t" << CheckY << "\t" << Potential << std::endl;

    if (Potential > MaxPotential) {  // 経路中でもっとも高いポテンシャルを計算（最大値計算）
      MaxPotential = Potential;
      // MaxPoint.x = CheckX;
      // MaxPoint.y = CheckY;
    }

  }
  // std::cout << "MaxPotential = " << MaxPotential << std::endl;
  if(MaxPotential > Threshold){
    return false;
  }else{
    return true;
  }
}


// f(x,y)
double MotionPlan::f_xy(double x,double y, std::vector<POINT> &vobstacle)
{
  double sum = 0.0;

  for (size_t i = 0, size = vobstacle.size(); i < size; ++i){
    sum += K*exp(-r_1*pow(x-vobstacle[i].x, 2) - r_2*pow(y-vobstacle[i].y, 2));
  }
  return sum;
}


// childrenのsubtreesの中で(xSample, ySample, zSample)に一番近いノードを探す(再帰的に)
// sample点から一番近いノードと距離をリターン
MotionPlan::RRT::TreeNode* MotionPlan::RRT::TreeNode::nearestNode(double xSample, double ySample, double* nearestDist)
{
  (*nearestDist) = -1;
  double dist;
  double dx, dy;
  TreeNode* nearest = NULL;
  TreeNode* childNearest = NULL;

  // すべてのchildren's treesのノードのなかで最も近いものを探す！
  std::vector<TreeNode*>::iterator child;
  for (child = children.begin(); child != children.end(); ++child) {
    childNearest = (*child)->nearestNode(xSample, ySample, &dist);

    // 今のやつが以前の最も近かったノードよりも近かったら
    if (dist < (*nearestDist) || nearest == NULL){
      nearest = childNearest;
      (*nearestDist) = dist;
    }
  }

  //いまのノードと比較
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
                     root(NULL),
                     xMin(NULL), xMax(NULL),
                     yMin(NULL), yMax(NULL)
{
  initFromFile(fileName);
  CreatePotentialField();
  std::ofstream file("./plot_data/testcase1_obstacle.dat");
  CreateCube(file);
  srand((unsigned int)time(NULL));
}


void MotionPlan::RRT::CreatePotentialField()
{
  POINT tmp;

  std::cout << "ポテンシャル場の作成" << std::endl;
  for (int i = 0; i < numObstacles; ++i) {
    for (double x = xMin[i]; x <= xMax[i]; ++x) {
      for(double y = yMin[i]; y <= yMax[i]; ++y){
        tmp.x = x; tmp.y = y;
        vobstacle.push_back(tmp);
      }
    }
  }
  // T-RRT用の定数を計算
  KConstant = 1000*(f_xy(xStart, yStart, vobstacle) + f_xy(xGoal, yGoal, vobstacle))/2.0;
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

  printf("\nフィールドの定義域は: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf]\n", xLeft, xRight, yBottom, yTop);

  cout << "障害物リスト" << endl;
  for (int i = 0; i < numObstacles; ++i){
    printf("             障害物%d: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf]\n", i+1, xMin[i], xMax[i], yMin[i], yMax[i]);
  }

  printf("\nスタートとゴール    : Start[%5.2lf, %5.2lf]\n", xStart, yStart);
  printf("                        End[%5.2lf, %5.2lf]\n\n", xGoal, yGoal);
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
}


// For x and y, generates a random number, scales it to [0,1], then
// scales it to the distance between the boundaries, then offsets it
// by the left/bottom value. This effectively generates x and y values
// in [xLeft, xRight] and [yBottom, yTop], respectively. We continue
// generating points until we find one that is clear of all obstacles.
void MotionPlan::RRT::randFreeSample(double* x, double* y)
{
  int roulette;
  roulette = (((double)rand())/RAND_MAX)*100;
  if(roulette >= 90){
    (*x) = xGoal;
    (*y) = yGoal;
  }else{
    do{
      (*x) = (((double)rand())/RAND_MAX)*(xRight - xLeft) + xLeft;
      (*y) = (((double)rand())/RAND_MAX)*(yTop - yBottom) + yBottom;
    } while(!clear(*x, *y, vobstacle));
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
{
  double dx = x - nearest->x;
  double dy = y - nearest->y;
  double dist = sqrt(dx * dx + dy * dy);

  double newX = nearest->x + stepSize * (dx / dist);
  double newY = nearest->y + stepSize * (dy / dist);

  if (link(nearest->x, nearest->y, newX, newY, vobstacle, stepSize)){

    TreeNode* newNode = new TreeNode;
    newNode->x = newX;
    newNode->y = newY;

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

  if ((dx*dx + dy*dy) <= stepSize*stepSize){
    return link(checkNode->x, checkNode->y, xGoal, yGoal, vobstacle, stepSize);
  } else{
    return false;
  }
}


bool MotionPlan::RRT::transitionTest(const TreeNode* child, const TreeNode* parent)
{
  double distance;
  double childCost, parentCost;

  cout << "KConstant = " << KConstant << endl;
  // KConstant = 0.5;

  // distance = sqrt(pow((child->x - parent->x), 2) + pow((child->y - parent->y), 2));
  distance = stepSize;
  childCost = f_xy(child->x, child->y, vobstacle);
  parentCost = f_xy(parent->x, parent->y, vobstacle);
  cout << "childCost = " << childCost << ", parentCost = " << parentCost << endl;

  // Always accept if new state has same or lower cost than old state
  if (childCost <= parentCost){
    cout << "transitionProbabilityは計算しなくていい" << endl;
    return true;
  }

  // Difference in cost
  double costSlope = (childCost - parentCost) / distance;
  cout << "costSlope = " << costSlope << endl;

  // The probability of acceptance of a new configuration is defined by comparing its cost c_j
  // relatively to the cost c_i of its parent in the tree. Based on the Metropolis criterion.
  double transitionProbability = 1.; // if cost_slope is <= 0, probabilty is 1

  // Only return at end
  bool result = false;

  // Calculate tranision probabilty
  if (costSlope > 0){
    transitionProbability = exp(-costSlope / (KConstant * Temperature));
    cout << "transitionProbability = " << transitionProbability << endl;
  }

  // Check if we can accept it
  double rand01;
  rand01 = ((double)rand())/RAND_MAX;
  cout << "rand01 = " << rand01 << endl;
  if (rand01 <= transitionProbability){
    if (Temperature > minTemperature){
      cout << "温度下げた！" << endl;
      Temperature /= tempChangeFactor;
      // Prevent temp_ from getting too small
      if (Temperature <= minTemperature) {
        Temperature = minTemperature;
      }
    }

    numStatesFailed = 0;

    result = true;
  } else {
    // State has failed
    if (numStatesFailed >= maxStatesFailed) {
      cout << "温度上げた！" << endl;
      Temperature *= tempChangeFactor;
      numStatesFailed = 0;
    } else {
      ++numStatesFailed;
    }
  }

  return result;
}


bool MotionPlan::RRT::minExpansionControl(double randMotionDistance)
{
  // Decide to accept or not
  if (randMotionDistance > frontierThreshold) {
    // participates in the tree expansion
    ++frontierCount;
    return true;
  } else {
    // participates in the tree refinement
    // check our ratio first before accepting it
    if ((double)nonfrontierCount / (double)frontierCount > frontierNodeRatio){
      // Increment so that the temperature rises faster
      ++numStatesFailed;
      // reject this node as being too much refinement
      return false;
    }else{
      ++nonfrontierCount;
      return true;
    }
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

    double sampleX, sampleY;
    TreeNode* near = NULL;
    TreeNode* newNode = NULL;

    randFreeSample(&sampleX, &sampleY);
    //std::cout << "(sampleX, sampleY, sampleZ) = (" << sampleX << ", " << sampleY << ", " << sampleZ  << " )"<< std::endl;

    near = nearestNode(sampleX, sampleY);
    newNode = genNewNode(near, sampleX, sampleY);

    if(newNode != NULL && transitionTest(near, newNode)){
      // if (newNode != NULL){
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
      // }
    }

  } // end while goal hasn't been reached
  printf("RRTは    %5dループで終了\n", (*iterations));

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


void MotionPlan::RRT::RRTloop(int* iterations, int* nodePath, int* pathLength, std::ostream& nodeData)
{
  std::string a;
  POINT tmp;

  while(1){
    if (findPath(iterations, nodePath, pathLength)) {
      outputTree(nodeData);

      for (int i = 0; i < (*pathLength); ++i) {
        tmp.x = (nodes[nodePath[i]])->x;
        tmp.y = (nodes[nodePath[i]])->y;
        paths.push_back(tmp);
      }

      std::ofstream pathData("./plot_data/path_data.dat", std::ios_base::trunc);
      for(unsigned int addpath = 0; addpath < paths.size(); addpath++ ){
        #ifdef APF
        pathData << paths[addpath].x << "\t" << paths[addpath].y << "\t" << MotionPlan::f_xy(paths[addpath].x, paths[addpath].y, vobstacle) << std::endl;
        #else
        pathData << paths[addpath].x << "\t" << paths[addpath].y << std::endl;
        #endif
      }
      break;

    } else {
      std::cout << "Path not found." << std::endl;
    }
  }
  smoothing(50000);

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
      dis += sqrt(pow(paths[i+1].x - paths[i].x, 2) +
                  pow(paths[i+1].y - paths[i].y, 2) );
    }
    // cout << "パスの距離は" << dis << "です。" << endl;
    return dis;
  }
}


void MotionPlan::RRT::smoothing(int loop)
{
  int SamplePoint[2];
  int tmp;
  int i;
  double ini;
  int initPathNum;
  double current, old = 0.0;
  int count = 0;

  ini = Distance();
  initPathNum = paths.size();
  //std::cout << "初期のパスの距離は" << ini << std::endl;

  for (i = 0; i < loop; ++i){
    if(paths.size()==2){ // もしパスの長さが2だったらもう間引けないからブレイク
      break;
    }

    while (1) { // ランダムサンプリングの2点を抽出
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
      }// else {
        //cout << "もう一回引き直し(ΦωΦ)" << endl;
      // }
    }

    // 2点を結んだ直線の干渉チェック
    if (link(paths[SamplePoint[0]].x, paths[SamplePoint[0]].y,
             paths[SamplePoint[1]].x, paths[SamplePoint[1]].y,
             vobstacle, stepSize)){
      //std::cout << SamplePoint[0] << "と" << SamplePoint[1] << "の間の点はグッバイ！" << std::endl;
      //std::cout << i+1 << "ループ目、グッバイしたあとのpathの長さは" << paths.size() << "です。" << std::endl;
      paths.erase(paths.begin()+SamplePoint[0]+1, paths.begin()+SamplePoint[1]);

      std::ofstream outStream("./plot_data/path_data_mod.dat", std::ios_base::trunc);
      for (unsigned int k = 0; k < paths.size(); k++) {
        #ifdef APF
        outStream << paths[k].x << "\t" << paths[k].y << "\t" << MotionPlan::f_xy(paths[k].x, paths[k].y, vobstacle) << std::endl;
        #else
        outStream << paths[k].x << "\t" << paths[k].y << std::endl;
        #endif
      }

      #ifdef PlotAnimation
      for (int k = 0; k < 1000; ++k){
        usleep(1000);
      }
      #endif

    } // else {
       // std::cout << i+1 << "ループ目はグッバイできませんでした。" << std::endl;
    // }
    current = Distance();
    //std::cout << "現在のパスの総距離は" << current << " 1ループ前のパスの総距離は" << old << std::endl;

    if( fabs(old - current) == 0.0 ){
      //std::cout << count + 1 << "回目、しきい値以下になりました！" << std::endl;
      count++;
    } else {
      //std::cout << "ブレイクカウントをリセット" << std::endl;
      count = 0;
    }

    old = current;
    if(count >= 20){
      //std::cout << "我慢ならん！ブレイクだ！！" << std::endl;
      break;
    }

  }
  printf("平滑化は %5dループで終了\n\n", i);
  printf("グッバイする前のパスの総距離は %5.3lf\n", ini);
  printf("        した後のパスの総距離は %5.3lf\n", Distance());
  printf("                               %5.3lf%%のダイエットに成功\n", (ini-Distance())/ini*100);
  printf("グッバイする前のパスの総数は %3d\n", initPathNum);
  printf("        した後のパスの総数は %3ld\n", paths.size());
}


void MotionPlan::RRT::CreateCube(std::ostream &cube)
{
  #ifndef APF
  RANGE obstacle;
  #endif
  // std::ofstream cube("./plot_data/testcase1_obstacle.dat");
  std::ofstream plot("./plot_data/start_goal.dat");
  std::ofstream pathinit("./plot_data/path_data.dat", std::ios_base::trunc);
  std::ofstream smoothData("./plot_data/path_data_mod.dat", std::ios_base::trunc);
  std::ofstream splineData("./plot_data/Bspline.dat", std::ios_base::trunc);

  #ifdef APF
  plot << xStart << "\t" << yStart << "\t" << MotionPlan::f_xy(xStart, yStart, vobstacle) << std::endl;
  plot << xGoal << "\t" << yGoal << "\t" << MotionPlan::f_xy(xGoal, yGoal, vobstacle) << std::endl;
  output_plt("./plot_data/APFplot.plt");
  #else
  plot << xStart << "\t" << yStart << std::endl;
  plot << xGoal << "\t" << yGoal << std::endl;

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
  #endif

  #ifdef PlotAnimation
  pathinit << xStart << "\t" << yStart << std::endl;
  pathinit << xStart << "\t" << yStart << std::endl;

  smoothData << xStart << "\t" << yStart << std::endl;
  smoothData << xStart << "\t" << yStart << std::endl;

  splineData << xStart << "\t" << yStart << std::endl;
  splineData << xStart << "\t" << yStart << std::endl;
  #endif

}


void MotionPlan::RRT::outputTree(std::ostream &outStream)
{
  #ifdef APF
  POINT tmp[2];
  #endif

  //ノードの座標を2点ずつのブロックでファイルに書き込み
  for (unsigned int i = 0; i < edges.size(); ++i){
    #ifdef APF
    tmp[0].x = (nodes[edges[i].node1])->x;
    tmp[0].y = (nodes[edges[i].node1])->y;
    tmp[1].x = (nodes[edges[i].node2])->x;
    tmp[1].y = (nodes[edges[i].node2])->y;
    outStream << tmp[0].x << "\t" << tmp[0].y << "\t" << MotionPlan::f_xy(tmp[0].x, tmp[0].y, vobstacle) << std::endl;
    outStream << tmp[1].x << "\t" << tmp[1].y << "\t" << MotionPlan::f_xy(tmp[1].x, tmp[1].y, vobstacle) << std::endl;
    #else
    outStream << (nodes[edges[i].node1])->x << "\t" << (nodes[edges[i].node1])->y << std::endl;
    outStream << (nodes[edges[i].node2])->x << "\t" << (nodes[edges[i].node2])->y << std::endl;
    #endif

    outStream << "\n" << std::endl;

    #ifdef PlotAnimation
    for (int k = 0; k < 100; ++k){
      usleep(100);
    }
    #endif

  }
}


void MotionPlan::RRT::output_plt(string plt_output){
  std::ofstream plt(plt_output);

  plt << "set xlabel \"x\""<< endl;
  plt << "set ylabel \"y\"" << endl;
  plt << "set zlabel \"z\"" << endl;
  plt << "set xrange [" << xLeft << ":" << xRight << "]" << endl;
  plt << "set yrange [" << yBottom << ":" << yTop << "]" << endl;
  plt << "set zrange [" << 0 << ":" << 2.5*K << "]" << endl;
  //plt << "set zrange [" << 0 << ":" << MAX_FIELD+1 << "]" << endl;
  plt << "set ticslevel 0\n" << endl;

  plt << "splot ";
  for (size_t i = 0, size = vobstacle.size(); i < size; ++i){
    plt << " + " << K << " * exp(-" << r_1 << " * (x-" << vobstacle[i].x << ")**2 - " << r_2 << " * (y-" << vobstacle[i].y << ")**2)";
  }

  plt << " title \"Potential Field\" with pm3d,\\" << endl;

  plt << "\"start_goal.dat\" using 1:2:3 with points pt 7 ps 2 lt rgb \"#ff9900\" title \"Start & Goal\",\\" << endl;
  plt << "\"data.dat\" using 1:2:3 with lines lt rgb \"#696969\" lw 1 title \"node\",\\" << endl;
  plt << "\"path_data.dat\" using 1:2:3 with lines lt rgb \"#ff4500\" lw 3 title \"Before\",\\" << endl;
  plt << "\"path_data_mod.dat\" using 1:2:3 with lines lt rgb \"#66cdaa\" lw 3 title \"After\"" << endl;
}


void MotionPlan::RRT::OutputFinalPath(std::vector<POINT> *finalpath)
{
  //std::cout << "copy" << std::endl;
  finalpath->resize(paths.size());
  copy(paths.begin(), paths.end(), finalpath->begin());
}
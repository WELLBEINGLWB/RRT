#include "MotionPlan.h"

/// Simple iterates over each object in the space
/// and checks whether the test point lies inside the obstacle.
bool MotionPlan::RRT::clear(double xTest, double yTest, double zTest){
  if(f_xy(xTest, yTest, zTest) > Threshold){
    return false;
  }else{
    return true;
  }
}



bool MotionPlan::RRT::link(double xStart, double yStart, double zStart,
                      double xDest, double yDest, double zDest,
                      double stepSize)
{
  double dx = xDest - xStart;
  double dy = yDest - yStart;
  double dz = zDest - zStart;
  double dist = sqrt(dx * dx + dy * dy + dz * dz);

  double CheckX;
  double CheckY;
  double CheckZ;

  double Potential;
  double MaxPotential = 0.0;
  // POINT MaxPoint;
  // bool flag = true;
  std::ofstream outStream("./plot_data/PotentialData.dat");

  if (!clear(xStart, yStart, zStart) ||
      !clear(xDest, yDest, zDest)) {
    //std::cout << "スタートとゴールが障害物内" << std::endl;
    return false;
  }

  for(double length = 0.0; length < dist; length += 0.5*stepSize){
    CheckX = xStart + length * (dx / dist);
    CheckY = yStart + length * (dy / dist);
    CheckZ = zStart + length * (dz / dist);
    Potential = f_xy(CheckX, CheckY, CheckZ);
    //outStream << CheckX << "\t" << CheckY << "\t" << Potential << std::endl;

    if (Potential > MaxPotential) {  // 経路中でもっとも高いポテンシャルを計算（最大値計算）
      MaxPotential = Potential;
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
double MotionPlan::RRT::f_xy(double x,double y, double z)
{
  double sum = 0.0;

  for (size_t i = 0, size = vobstacle.size(); i < size; ++i){
    sum += K*exp(-r_1*pow(x-vobstacle[i].x, 2) - r_2*pow(y-vobstacle[i].y, 2) - r_3*pow(z-vobstacle[i].z, 2));
  }
  return sum + K_1*(pow(x-xGoal, 2) + pow(y-yGoal, 2) + pow(z-zGoal, 2));
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
  CreatePotentialField();
  std::ofstream file("./plot_data/testcase1_obstacle.dat");
  CreateCube(file);
  srand((unsigned int)time(NULL));
  cout << "コンストラクタ終了" << endl;
}



void MotionPlan::RRT::CreatePotentialField()
{
  POINT tmp;

  std::cout << "ポテンシャル場の作成" << std::endl;
  for (int i = 0; i < numObstacles; ++i) {
    for (double x = xMin[i]; x <= xMax[i]; x+=0.1) {
      for(double y = yMin[i]; y <= yMax[i]; y+=0.1){
        for(double z = zMin[i]; z <= zMax[i]; z+=0.1){
          tmp.x = x; tmp.y = y; tmp.z = z;
          vobstacle.push_back(tmp);
        }
      }
    }
  }
  // T-RRT用の定数を計算
  KConstant = 10000*(f_xy(xStart, yStart, zStart) + f_xy(xGoal, yGoal, zGoal))/2.0;
}



void MotionPlan::RRT::Evaluation(int num){
  double dx,dy;
  POINT newP;
  std::vector<POINT> DigitalPoint;
  double MaxCost = 0.0;
  double AveCost = 0.0;
  double SumCost = 0.0;
  double CostCurrent, CostOld, CostDiff;
  double W, Wc = 0.0, Wd = 0.0;
  double d;
  double S_sum = 0.0, sigma;

  for(unsigned int i = 0; i < paths.size()-1; i++ ){
    dx = paths[i+1].x - paths[i].x;
    dy = paths[i+1].y - paths[i].y;
    for (int j = 0; j < num; ++j){
      newP.x = paths[i].x + j * (dx / num);
      newP.y = paths[i].y + j * (dy / num);
      DigitalPoint.push_back(newP);
    }
  }
  newP.x = paths[paths.size()-1].x;
  newP.y = paths[paths.size()-1].y;
  DigitalPoint.push_back(newP);

  std::ofstream plot("./plot_data/digitaldata.dat");
  for (unsigned int i = 0; i < DigitalPoint.size(); i++ ){
    CostCurrent = f_xy(DigitalPoint[i].x, DigitalPoint[i].y, DigitalPoint[i].z);
    // コストの合計
    SumCost += CostCurrent;
    // コストの最大値計算
    if(CostCurrent > MaxCost){
      MaxCost = CostCurrent;
    }
    // JailletのW(p)
    if(i > 0){
      CostDiff = CostCurrent - CostOld;
      if(CostDiff <= 0){
        CostDiff = 0;
      }
      d = sqrt(pow(DigitalPoint[i].x-DigitalPoint[i-1].x, 2) + pow(DigitalPoint[i].y-DigitalPoint[i-1].y, 2));
      Wc += CostDiff * d;
      Wd += d;
    }
    plot << DigitalPoint[i].x << "\t" << DigitalPoint[i].y << "\t" << CostCurrent << std::endl;
    CostOld = CostCurrent;
  }
  // JailletのW(p)
  W = Wc + 0.001 * stepSize * Wd;

  // コストの平均値
  AveCost = SumCost / DigitalPoint.size();

  // コストの標準偏差
  for (unsigned int i = 0; i < DigitalPoint.size(); i++ ){
    S_sum += pow((f_xy(DigitalPoint[i].x, DigitalPoint[i].y, DigitalPoint[i].z) - AveCost),2);
  }
  sigma = sqrt(S_sum/DigitalPoint.size());

  // 評価データを保存
  time_t t = time(NULL);
  strftime(savefilename, sizeof(savefilename), "./evaluation_data/T-RRT/Evaluate_20%y.%m.%d_%H:%M:%S.dat", localtime(&t));
  std::ofstream data(savefilename);
  data << "Length = " << Wd << endl;
  data << "MaxCost = " << MaxCost << endl;
  data << "AveCost = " << AveCost << endl;
  data << "SumCost = " << SumCost << endl;
  data << "W(p) = " << W << endl;
  data << "S = " << sigma << endl;
  data << "Num of Point = " << paths.size() << endl;

  cout << "Length = " << Wd << endl;
  cout << "MaxCost = " << MaxCost << endl;
  cout << "AveCost = " << AveCost << endl;
  cout << "SumCost = " << SumCost << endl;
  cout << "W(p) = " << W << endl;
  cout << "S = " << sigma << endl;
  cout << "Num of Point = " << paths.size() << endl;
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

  printf("\nフィールドの定義域は: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf] z[%5.2lf, %5.2lf]\n", xLeft, xRight, yBottom, yTop, zBottom, zTop);

  cout << "障害物リスト" << endl;
  for (int i = 0; i < numObstacles; ++i){
    printf("             障害物%d: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf] z[%5.2lf, %5.2lf]\n", i+1, xMin[i], xMax[i], yMin[i], yMax[i], zMin[i], zMax[i]);
  }

  printf("\nスタートとゴール    : Start[%5.2lf, %5.2lf, %5.2lf]\n", xStart, yStart, zStart);
  printf("                        End[%5.2lf, %5.2lf, %5.2lf]\n\n", xGoal, yGoal, zGoal);
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
  int roulette;
  roulette = (((double)rand())/RAND_MAX)*100;
  if(roulette >= 90){
    (*x) = xGoal;
    (*y) = yGoal;
    (*z) = zGoal;
  }else{
    do{
      (*x) = (((double)rand())/RAND_MAX)*(xRight - xLeft) + xLeft;
      (*y) = (((double)rand())/RAND_MAX)*(yTop - yBottom) + yBottom;
      (*z) = (((double)rand())/RAND_MAX)*(zTop - zBottom) + zBottom;
    } while(!clear(*x, *y, *z));
  }
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

  if (link(nearest->x, nearest->y, nearest->z, newX, newY, newZ, stepSize)){

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
    return link(checkNode->x, checkNode->y, checkNode->z, xGoal, yGoal, zGoal, stepSize);
  } else{
    return false;
  }
}



bool MotionPlan::RRT::transitionTest(const TreeNode* child, const TreeNode* parent)
{
  double distance;
  double childCost, parentCost;

  // KConstantはスタートとゴールのポテンシャルによってきまる
  #ifdef ChecktransitionTest
  cout << "KConstant = " << KConstant << endl;

  cout << "Temperature = " << Temperature << endl;
  #endif

  distance = stepSize;
  // 親ノードと子ノードのコスト計算
  childCost = f_xy(child->x, child->y, child->z);
  parentCost = f_xy(parent->x, parent->y, child->z);
  #ifdef ChecktransitionTest
  cout << "childCost = " << childCost << ", parentCost = " << parentCost << endl;
  #endif

  // もし，親ノードより子ノードがコストが低かったら
  if (childCost <= parentCost){
    #ifdef ChecktransitionTest
    cout << "transitionProbabilityは計算しなくていい" << endl;
    #endif
    return true;
  }

  // コストの差と距離を計算
  double costSlope = (childCost - parentCost) / distance;
  #ifdef ChecktransitionTest
  cout << "costSlope = " << costSlope << endl;
  #endif
  double transitionProbability = 1.0; // if cost_slope is <= 0, probabilty is 1

  // falseで初期化
  bool result = false;

  // 確率計算
  if (costSlope > 0){
    transitionProbability = exp(-costSlope / (KConstant * Temperature));
    #ifdef ChecktransitionTest
    cout << "transitionProbability = " << transitionProbability << endl;
    #endif
  }

  // Check if we can accept it
  double rand01;
  rand01 = ((double)rand())/RAND_MAX;
  #ifdef ChecktransitionTest
  cout << "rand01 = " << rand01 << endl;
  #endif
  if (rand01 <= transitionProbability){
    if (Temperature > minTemperature){
      #ifdef ChecktransitionTest
      cout << "温度下げた！" << endl;
      #endif
      Temperature /= tempChangeFactor;
      // Temperatureが小さ過ぎたら
      if (Temperature <= minTemperature) {
        Temperature = minTemperature;
      }
    }

    numStatesFailed = 0;

    result = true;
  } else {
    // State has failed
    if (numStatesFailed >= maxStatesFailed) {
      #ifdef ChecktransitionTest
      cout << "温度上げた！" << endl;
      #endif
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
    //std::cout << "(sampleX, sampleY, sampleZ) = (" << sampleX << ", " << sampleY << ", " << sampleZ  << " )"<< std::endl;

    near = nearestNode(sampleX, sampleY, sampleZ);
    newNode = genNewNode(near, sampleX, sampleY, sampleZ);

    // if (newNode != NULL && transitionTest(near, newNode)){
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
  POINT tmp;

  while(1){
    if (findPath(iterations, nodePath, pathLength)) {
      outputTree(nodeData);

      for (int i = 0; i < (*pathLength); ++i) {
        tmp.x = (nodes[nodePath[i]])->x;
        tmp.y = (nodes[nodePath[i]])->y;
        tmp.z = (nodes[nodePath[i]])->z;
        paths.push_back(tmp);
      }

      std::ofstream pathData("./plot_data/path_data.dat", std::ios_base::trunc);
      for(unsigned int addpath = 0; addpath < paths.size(); addpath++ ){
        pathData << paths[addpath].x << "\t" << paths[addpath].y << "\t" << paths[addpath].z << std::endl;
      }
      break;

    } else {
      std::cout << "Path not found." << std::endl;
    }
  }
  #ifdef Evaluate
  Evaluation(1);
  #endif
  #ifdef Smooth
  smoothing(10000);
  #endif
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
                  pow(paths[i+1].y - paths[i].y, 2) +
                  pow(paths[i+1].z - paths[i].z, 2) );
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
    if (link(paths[SamplePoint[0]].x, paths[SamplePoint[0]].y, paths[SamplePoint[0]].z,
             paths[SamplePoint[1]].x, paths[SamplePoint[1]].y, paths[SamplePoint[1]].z,
             stepSize)){
      //std::cout << SamplePoint[0] << "と" << SamplePoint[1] << "の間の点はグッバイ！" << std::endl;
      //std::cout << i+1 << "ループ目、グッバイしたあとのpathの長さは" << paths.size() << "です。" << std::endl;
      paths.erase(paths.begin()+SamplePoint[0]+1, paths.begin()+SamplePoint[1]);

      std::ofstream outStream("./plot_data/path_data_mod.dat", std::ios_base::trunc);
      for (unsigned int k = 0; k < paths.size(); k++) {
        outStream << paths[k].x << "\t" << paths[k].y << "\t" << paths[k].z << std::endl;
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
  RANGE obstacle;
  std::ofstream plot("./plot_data/start_goal.dat");
  std::ofstream pathinit("./plot_data/path_data.dat", std::ios_base::trunc);
  std::ofstream smoothData("./plot_data/path_data_mod.dat", std::ios_base::trunc);
  std::ofstream splineData("./plot_data/Bspline.dat", std::ios_base::trunc);

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

  #ifdef PlotAnimation
  pathinit << xStart << "\t" << yStart << zStart << std::endl;
  pathinit << xStart << "\t" << yStart << zStart << std::endl;

  smoothData << xStart << "\t" << yStart << zStart << std::endl;
  smoothData << xStart << "\t" << yStart << zStart << std::endl;

  splineData << xStart << "\t" << yStart << zStart << std::endl;
  splineData << xStart << "\t" << yStart << zStart << std::endl;
  #endif

}



void MotionPlan::RRT::outputTree(std::ostream &outStream)
{
  //ノードの座標を2点ずつのブロックでファイルに書き込み
  for (unsigned int i = 0; i < edges.size(); ++i){
    outStream << (nodes[edges[i].node1])->x << "\t" << (nodes[edges[i].node1])->y << "\t" << (nodes[edges[i].node1])->z << std::endl;
    outStream << (nodes[edges[i].node2])->x << "\t" << (nodes[edges[i].node2])->y << "\t" << (nodes[edges[i].node2])->z << std::endl;
    outStream << "\n" << std::endl;

    #ifdef PlotAnimation
    for (int k = 0; k < 100; ++k){
      usleep(100);
    }
    #endif

  }
}



void MotionPlan::RRT::outputTree(FILE *outStream)
{
  for (unsigned int i = 0; i < edges.size(); ++i){
    std::cout << i << "番目のループだよ" << std::endl;
    //fprintf(outStream, "plot '-' using 1:2 with lines");

    fprintf(outStream,"%lf %lf %lf\n", ((nodes[edges[i].node1])->x), ((nodes[edges[i].node1])->y), ((nodes[edges[i].node1])->z));
    fprintf(outStream,"%lf %lf %lf\n\n", ((nodes[edges[i].node2])->x), ((nodes[edges[i].node2])->y), ((nodes[edges[i].node2])->z));
    fprintf(outStream,"e\n"); //データ書き込み終了を知らせる．
    //}
    fflush(outStream);


    // for(int k=0; k<50; ++k){
    //   usleep(10000);
    // }

  }
}



void MotionPlan::RRT::outputPotential(std::ostream &outStream)
{
  for (int i = 0; i < numObstacles; ++i) {
    for (double x = xMin[i]-1; x <= xMax[i]+1; x+=0.05) {
      for(double y = yMin[i]-1; y <= yMax[i]+1; y+=0.05){
        for(double z = zMin[i]; z <= zMax[i]+1; z+=0.1){
          outStream << x << "\t" << y << "\t" << z << "\t" << f_xy(x, y, z) << endl;
        }
      }
    }
  }

}



void MotionPlan::RRT::OutputFinalPath(std::vector<POINT> *finalpath)
{
  //std::cout << "copy" << std::endl;
  finalpath->resize(paths.size());
  copy(paths.begin(), paths.end(), finalpath->begin());
}
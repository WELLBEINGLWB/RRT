#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>

#define K 60      // 障害物のポテンシャルの高さ
#define r_1 3     // ポテンシャルのx軸方向の大きさ
#define r_2 3     // ポテンシャルのy軸方向の大きさ
#define Threshold 30

using namespace std;

typedef struct{
  double x;
  double y;
} POINT;

typedef struct{
  double xrange[2];
  double yrange[2];
} RANGE;
vector<POINT> vobstacle;

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


void initFromFile(std::string fileName)
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
  std::ofstream cube("./plot_data/testcase1_obstacle.dat");
  RANGE obstacle;

  Start_and_Goal << xStart << "\t" << yStart << std::endl;
  Start_and_Goal << xGoal << "\t" << yGoal << std::endl;

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

  printf("\nフィールドの定義域は: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf]\n", xLeft, xRight, yBottom, yTop);

  cout << "障害物リスト" << endl;
  for (int i = 0; i < numObstacles; ++i){
    printf("             障害物%d: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf]\n", i+1, xMin[i], xMax[i], yMin[i], yMax[i]);
  }

  printf("\nスタートとゴール    : Start[%5.2lf, %5.2lf]\n", xStart, yStart);
  printf("                        End[%5.2lf, %5.2lf]\n\n", xGoal, yGoal);
}


// f(x,y)
double f_xyz(double x,double y)
{
  double sum = 0.0;

  for (size_t i = 0, size = vobstacle.size(); i < size; ++i){
    sum += K*exp(-r_1*pow(x-vobstacle[i].x, 2) - r_2*pow(y-vobstacle[i].y, 2));
  }
  return sum;
}


void CreatePotentialField()
{
  POINT tmp;

  cout << "ポテンシャル場の作成" << endl;
  for (int i = 0; i < numObstacles; ++i) {
    for (double x = xMin[i]; x <= xMax[i]; ++x) {
      for(double y = yMin[i]; y <= yMax[i]; ++y){
        tmp.x = x; tmp.y = y;
        vobstacle.push_back(tmp);
      }
    }
  }
}


bool clear(double xTest, double yTest){
  if(f_xyz(xTest, yTest) > Threshold){
    return false;
  }else{
    return true;
  }
}


bool Link(double xStart, double yStart,
          double xDest, double yDest)
{
  double dx = xDest - xStart;
  double dy = yDest - yStart;
  double dist = sqrt(dx * dx + dy * dy);
  double stepSize = 0.1;

  double CheckX;
  double CheckY;

  double Potential;
  double MaxPotential = 0.0;
  POINT MaxPoint;
  // bool flag = true;
  std::ofstream outStream("./plot_data/PotentialData.dat");

  if (!clear(xStart, yStart) ||
      !clear(xDest, yDest)) {
    cout << "スタートとゴールが障害物内" << endl;
    return false;
  }

  for(double length = 0.0; length < dist; length += 0.1*stepSize){
    CheckX = xStart + length * (dx / dist);
    CheckY = yStart + length * (dy / dist);
    Potential = f_xyz(CheckX, CheckY);
    outStream << CheckX << "\t" << CheckY << "\t" << Potential << endl;

    if (Potential > MaxPotential) {  // 経路中でもっとも高いポテンシャルを計算（最大値計算）
      MaxPotential = Potential;
      MaxPoint.x = CheckX; MaxPoint.y = CheckY;
    }

  }
  cout << "MaxPotential = " << MaxPotential << endl;
  if(MaxPotential > Threshold){
    return false;
  }else{
    return true;
  }
}


int main(){
  bool Collision;

  initFromFile("./plot_data/testcase1.dat");
  CreatePotentialField();

  Collision = Link(xStart, yStart, xGoal, yGoal);
  if(Collision == true){
    cout << "当たってない" << endl;
  }else{
    cout << "当たってる" << endl;
  }

}
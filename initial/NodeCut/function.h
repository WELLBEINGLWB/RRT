#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <iostream>
#include <fstream>
#include <ostream>

#include <iterator>
#include <vector>
#include <string>
#include <algorithm>

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <sys/time.h>
#include <unistd.h>

#define K 100      // 障害物のポテンシャルの高さ
#define K_1 4      // 引力のポテンシャルの高さ
#define r_1 2    // ポテンシャルのx軸方向の大きさ
#define r_2 2     // ポテンシャルのy軸方向の大きさ

using namespace std;

typedef struct{
  double x;
  double y;
} POINT;

typedef struct {
  double xrange[2];
  double yrange[2];
} RANGE;

bool input_arg(int argc, char* argv[], string filename[]);

class Evaluate {
  public:
    Evaluate(std::string fileName[], double InputDistance);
    int CountNumbersOfTextLines(string fileName);
    void initPathFromFile(string fileName);
    void initObstacleFromFile(std::string fileName);
    void CreatePotentialField();
    double f_xy(double x, double y);
    double Distance(POINT a, POINT b);
    double PathDistance(int max);
    void Split();
    void Eva();
    char savefilename[64] = {'\0'};

    void CreateCube(string obstacle_output);
    void output_plt(string plt_output, char* argv[]);

  private:
    int data_num;
    vector< double > D;
    vector< POINT > pathdata, node, obstacle;

    double inDistance;

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
};
#endif
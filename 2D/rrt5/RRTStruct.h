#ifndef RRT_STRUCT_H_
#define RRT_STRUCT_H_

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

// #define Bspline
// #define PlotAnimation
#define CalcPotential
// #define Smooth

using namespace std;

#define K 1      // 障害物のポテンシャルの高さ
#define K_1 0.001      // 引力のポテンシャルの高さ
#define r_1 2     // ポテンシャルのx軸方向の大きさ
#define r_2 2     // ポテンシャルのy軸方向の大きさ

typedef struct{
  double x;
  double y;
} POINT;

typedef struct {
  double xrange[2];
  double yrange[2];
} RANGE;

string input_arg(int argc, char* argv[]);

#endif

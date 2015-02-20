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

// #define PlotAnimation
// #define APF
#define Bspline
// #define Evaluate
#define Smooth

#define K 100        // 障害物のポテンシャルの高さ
// #define K 60        // （以前の）障害物のポテンシャルの高さ
#define K_1 1       // goalに導く引力ポテンシャルの高さ
#define r_1 2     // ポテンシャルのx軸方向の大きさ
#define r_2 2     // ポテンシャルのy軸方向の大きさ
#define r_3 2     // ポテンシャルのz軸方向の大きさ

using namespace std;

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

string input_arg(int argc, char* argv[]);

#endif

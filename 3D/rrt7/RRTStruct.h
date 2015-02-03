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
// #define Bspline
// #define Smooth
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

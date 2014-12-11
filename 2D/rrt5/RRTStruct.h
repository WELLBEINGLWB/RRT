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

using namespace std;

typedef struct{
  double x;
  double y;
} POINT;

typedef struct {
  double xrange[2];
  double yrange[2];
} RANGE;

#endif

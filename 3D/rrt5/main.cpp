#include <iostream>
#include <fstream>
#include "MotionPlan.h"
#include <sys/time.h>

int main(int argc, char* argv[])
{
  int iters;
  int path[4096];
  int pathLength;

  MotionPlan::RRT rrt("./plot_data/testcase1.dat");
  std::ofstream file("./plot_data/data.dat");

  struct timeval start, end;
  gettimeofday(&start, NULL);

  rrt.RRTloop(&iters, path, &pathLength, file);

  gettimeofday(&end, NULL);
  std::cout << "RRTの実行時間は " << (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)*1.0E-6 << "[s]でした。" << std::endl;

  return 0;
}

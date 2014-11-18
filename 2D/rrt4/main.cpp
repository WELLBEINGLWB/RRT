#include "MotionPlan.h"

int main(int argc, char* argv[])
{
  int iters;
  int path[1024];
  int pathLength;

  MotionPlan::RRT rrt("./plot_data/testcase3.dat");

  std::ofstream file("./plot_data/data.dat");
  //std::ofstream path_file("./plot_data/path_data.dat");

  rrt.RRTloop(&iters, path, &pathLength, file);

  return 0;
}

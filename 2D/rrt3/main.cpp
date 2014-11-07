#include <iostream>
#include <fstream>
#include "MotionPlan.h"

int main(int argc, char* argv[])
{
  int iters;
  int path[1024];
  int pathLength;
  int pathSum = 0;
  int nodeSum = 0;
  int itersSum = 0;

  MotionPlan::RRT rrt("./plot_data/testcase2o.dat");

  std::ofstream file("./plot_data/data.dat");

  std::ofstream path_file("./plot_data/path_data.dat");

  for (int i = 0; i < 1; ++i) {
    if (rrt.findPath(&iters, path, &pathLength)) {
      // file << "Iterations: " << iters << std::endl;
      itersSum += iters;

      rrt.outputTree(file);
      nodeSum += rrt.nodes.size();

      // file << "--- Path ---" << std::endl;
      for (int j = 0; j < pathLength; ++j) {
        path_file << (rrt.nodes[path[j]])->x << "\t" << (rrt.nodes[path[j]])->y << std::endl;
      }

      pathSum += pathLength;

    } else {
      std::cout << "Path not found." << std::endl;
    }
  }

  // file << "Average Comp. time: " << (itersSum / 20.0) << " iterations" << std::endl;
  // file << "Average # of nodes: " << (nodeSum / 20.0) << std::endl;
  // file << "Average path length: " << (pathSum / 20.0) << std::endl;

  file.close();
  path_file.close();

  return 0;
}

#include <iostream>
#include <fstream>
#include "MotionPlan.h"

int main(int argc, char* argv[])
{
  int iters;
  int path[4096];
  int pathLength;
  int pathSum = 0;
  int nodeSum = 0;
  int itersSum = 0;
  int i = 0;

  MotionPlan::RRT rrt("./plot_data/testcase1.dat");

  std::ofstream file("./plot_data/data.dat");
  std::ofstream path_file("./plot_data/path_data.dat");
  std::ofstream obstacle("./plot_data/testcase1_obstacle.dat");

  rrt.CreateCube(obstacle);

  while(i < 5){
    if (rrt.findPath(&iters, path, &pathLength)) {
      // file << "Iterations: " << iters << std::endl;
      itersSum += iters;

      rrt.outputTree(file);
      nodeSum += rrt.nodes.size();

      // file << "--- Path ---" << std::endl;
      for (int j = 0; j < pathLength; ++j) {
        path_file << (rrt.nodes[path[j]])->x << "\t" << (rrt.nodes[path[j]])->y << "\t" << (rrt.nodes[path[j]])->z << std::endl;
      }

      pathSum += pathLength;

      i++;
      break;
    } else {
      std::cout << i+1 << " iterations Path not found." << std::endl;
      i++;
    }
  }

  file.close();
  path_file.close();
  obstacle.close();
  return 0;
}

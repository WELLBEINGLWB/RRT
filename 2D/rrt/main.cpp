#include <iostream>
#include "MotionPlan.h"
#include <fstream>

int main(int argc, char* argv[]){
//   double obsXMin[] = {0.0};
//   double obsXMax[] = {1.0};
//   double obsYMin[] = {-1.0};
//   double obsYMax[] = {0.0};

//   double xTest = -1.0;
//   double yTest = -1.0;
//   double xDest = 1.0;
//   double yDest = 2.0;

//   std::cout << MotionPlan::clear(obsXMin, obsXMax, obsYMin, obsYMax, 1, xTest, yTest) << std::endl;
//   std::cout << MotionPlan::link(obsXMin, obsXMax, obsYMin, obsYMax, 1, xTest, yTest, xDest, yDest) << std::endl;

//   return 0;

  int iters;
  int path[1024];
  int pathLength;
  int pathSum = 0;
  int nodeSum = 0;
  int itersSum = 0;

  MotionPlan::RRT rrt("testcase1.dat");

  std::ofstream file("data.dat");
  std::ofstream path_file("path_data.dat");

  file << "****************************************" << std::endl;
  file << "Test Case 1" << std::endl;
  file << "****************************************" << std::endl;
  for (int i = 0; i < 1; ++i){

    if (rrt.findPath(&iters, path, &pathLength)){
      file << "Iterations: " << iters << std::endl;
      itersSum += iters;

      rrt.outputTree(file);
      nodeSum += rrt.nodes.size();

      file << "--- Path ---" << std::endl;

      for (int j = 0; j < pathLength; ++j)
        path_file << path[j] << std::endl;

      pathSum += pathLength;

    }
    else{
      std::cout << "Path not found." << std::endl;
    }
  }

  file << "Average Comp. time: " << (itersSum / 20.0) << " iterations" << std::endl;
  file << "Average # of nodes: " << (nodeSum / 20.0) << std::endl;
  file << "Average path length: " << (pathSum / 20.0) << std::endl;

  // rrt.initFromFile("testcase2.dat");
  // itersSum = 0;
  // nodeSum = 0;
  // pathSum = 0;

  // file << "****************************************" << std::endl;
  // file << "Test Case 2" << std::endl;
  // file << "****************************************" << std::endl;
  // for (int i = 0; i < 20; ++i)
  // {

  //   if (rrt.findPath(&iters, path, &pathLength))
  //   {
  //     file << "Iterations: " << iters << std::endl;
  //     itersSum += iters;

  //     rrt.outputTree(file);
  //     nodeSum += rrt.nodes.size();

  //     file << "--- Path ---" << std::endl;

  //     for (int j = 0; j < pathLength; ++j)
  //      file << path[j] << std::endl;

  //    pathSum += pathLength;

  //  }
  //  else
  //   file << "Path not found." << std::endl;
  // }
  // file << "Average Comp. time: " << (itersSum / 20.0) << " iterations" << std::endl;
  // file << "Average # of nodes: " << (nodeSum / 20.0) << std::endl;
  // file << "Average path length: " << (pathSum / 20.0) << std::endl;

  file.close();
  path_file.close();

  return 0;
}

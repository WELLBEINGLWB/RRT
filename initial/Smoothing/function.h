#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <iostream>
#include <ostream>
#include <fstream>
#include <string>

#include <vector>
#include <iterator>

#include <cmath>
#include <ctime>
#include <cstdlib>

#include <unistd.h>

using namespace std;

typedef struct{
  double x;
  double y;
} POINT;

class Smoothing
{
 public:
  enum LineType { HORIZONTAL, VERTICAL, GENERAL };

  Smoothing(std::string obstaclefile, std::string pathfile);
  void initFromFile(std::string fileName);
  int CountNumbersOfTextLines(std::string fileName);
  void Input_Data(std::string fileName);
  double Distance();
  bool equal(double x, double y);
  bool clear(const double* xMin, const double* xMax, const double* yMin, const double* yMax,
             int numObstacles, double xTest, double yTest);
  bool link(const double* xMin, const double* xMax, const double* yMin, const double* yMax,
            int numObstacles, double xStart, double yStart, double xDest, double yDest);
  int GetRandom(double min, double max);
  void PrintData();
  void OutputData();
  void PrintObstacle();
  void smoothing(int loop);
  void onestep_smoothing(int loop);

 private:
  int data_num;
  std::vector<POINT> paths;
  double* xMin;
  double* xMax;
  double* yMin;
  double* yMax;
  int numObstacles;
  double EPSILON = 1e-9;
};
#endif

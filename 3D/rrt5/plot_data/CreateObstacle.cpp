#include <iostream>
#include <fstream>
#include <ostream>
using namespace std;

int main()
{
  ofstream obstacle("./testcase1.dat");
  double xLeft = 0.0;
  double xRight = 20.0;
  double yBottom = 0.0;
  double yTop = 20.0;
  double zBottom = 0.0;
  double zTop = 20.0;
  int numObstacles = 10;

	double xStart = 1.0;
	double yStart = 1.0;
	double zStart = 1.0;
	double xGoal = 19.0;
	double yGoal = 19.0;
	double zGoal = 19.0;
	double stepSize = 0.4;


  obstacle << xLeft << "\n" << xRight << "\n" << yBottom << "\n" << yTop << "\n" << zBottom << "\n" << zTop << "\n" << numObstacles << endl;

  for (int i = 0; i < numObstacles; ++i) {
    obstacle << 10 - 0.5 * i << " " << 12 - 0.5 * i << " " << 10 - 0.5 * i << " " << 12 - 0.5 * i << " " << 2*i << " " << 2*(i + 1) << endl;
  }

   obstacle << xStart << " " << yStart << " " << zStart << "\n" << xGoal << " " << yGoal << " " << zGoal << "\n" << stepSize;
}
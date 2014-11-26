#include <iostream>
#include <cmath>
#include <fstream>
#include <ostream>
using namespace std;
#define MaxSplineSize 100

typedef struct{
  double x;
  double y;
  double z;
} POINT;

void drowSpline(double *x, double *y, double *z, int num);
class Spline
{
 private:
  int num;
  double a[MaxSplineSize + 1], b[MaxSplineSize + 1], c[MaxSplineSize + 1], d[MaxSplineSize + 1];

 public:
  Spline() { num = 0; }
  void init(double *sp, int num);
  double calc(double t);
};
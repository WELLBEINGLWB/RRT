#include "RRTStruct.h"
#define MaxSplineSize 100

void drowSpline(std::vector<POINT> &finalpath);
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
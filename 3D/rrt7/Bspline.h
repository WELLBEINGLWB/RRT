#include "RRTStruct.h"
#include "MotionPlan.h"
#define MaxSplineSize 500

class Draw
{
  public:
    Draw(MotionPlan::RRT &rrt);
    class Spline
    {
      public:
        Spline() { num = 0; }
        void init(double *sp, int num);
        double calc(double t);

      private:
        int num;
        double a[MaxSplineSize + 1], b[MaxSplineSize + 1], c[MaxSplineSize + 1], d[MaxSplineSize + 1];
    };
    void drowSpline(std::vector<POINT> &finalpath);
    void CreatePotentialField(MotionPlan::RRT &rrt);
    double f_xyz(double x,double y, double z);

  private:
    int num = 0;
    double *x;
    double *y;
    double *z;
    vector<POINT> SplinePoint;
    vector<POINT> vobstacle;

};
#include "Bspline.h"

int main(void){
  double x[5] = {2, 9, 5, 2, 8};
  double y[5] = {2, 8, 7, 6, 4};
  double z[5] = {2, 3, 12, 2.5, 8};

  drowSpline(x, y, z, 5);

  return 0;
}
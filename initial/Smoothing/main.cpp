#include "function.h"

int main(void){
  Smoothing smoo("./plot_data/testcase1.dat", "./plot_data/path_data.dat");
  smoo.PrintObstacle();
  smoo.smoothing();

  return 0;
}
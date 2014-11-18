#include "function.h"
using namespace std;
int main(void){
  Smoothing smoo("./plot_data/testcase1.dat", "./plot_data/path_data.dat");
  double before, after;

  smoo.PrintObstacle();
  before = smoo.Distance();

  smoo.smoothing(500000);

  smoo.OutputData();
  after = smoo.Distance();
  printf("グッバイする前のパスの総距離は%5.3lf\n", before);
  printf("        した後のパスの総距離は%5.3lf\n", after);

  return 0;
}
#include "RRTStruct.h"
#include "MotionPlan.h"
#include "Bspline.h"

int main(int argc, char* argv[])
{
  int iters;
  int path[4096];
  int pathLength;
  vector<POINT> finalpath;

  /*----------探索をするためのファイルをインプットする----------*/
  string inputfilename;
  inputfilename = input_arg(argc, argv);
  if(inputfilename == "ERROR"){
    return -1;
  }
  /*------------------------------------------------------------*/

  MotionPlan::RRT rrt(inputfilename); // RRTの計算クラス
  // Draw Sp(rrt); // B-splineの計算、APFチェックのクラス
  std::ofstream file("./plot_data/data.dat");


  #ifndef PlotAnimation
  struct timeval start, end;
  gettimeofday(&start, NULL);
  #endif

  rrt.RRTloop(&iters, path, &pathLength, file);
  rrt.OutputFinalPath(&finalpath);

  // Sp.drowSpline(finalpath);

  #ifndef PlotAnimation
  gettimeofday(&end, NULL);
  cout << "RRTの実行時間は " << (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)*1.0E-6 << "[s]でした。" << endl;
  #endif

  return 0;
}

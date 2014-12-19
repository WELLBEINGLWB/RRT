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

  MotionPlan::RRT rrt(inputfilename);         // RRTの計算クラス
  std::ofstream file("./plot_data/data.dat"); // 木の幹用ファイル作成（全部）

  #ifdef Bspline               // 経路の平滑化をするかどうか
  Draw Sp(rrt);                // B-splineの計算、APFチェックのクラス
  #endif

  #ifndef PlotAnimation
  struct timeval start, end;
  gettimeofday(&start, NULL);
  #endif

  rrt.RRTloop(&iters, path, &pathLength, file);
  rrt.OutputFinalPath(&finalpath);
  // std::ofstream potential("./plot_data/PotentialData.dat"); // 3次元データ＋ポテンシャルの４次元データ
  // rrt.outputPotential(potential);

  #ifdef Bspline
  Sp.drowSpline(finalpath);    // 平滑化＋ポテンシャル法による経路の衝突判定β
  #endif

  #ifndef PlotAnimation
  gettimeofday(&end, NULL);
  cout << "RRTの実行時間は " << (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)*1.0E-6 << "[s]でした。" << endl;
  #endif

  return 0;
}

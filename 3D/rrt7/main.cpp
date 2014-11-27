#include "RRTStruct.h"
#include "MotionPlan.h"
#include "Bspline.h"

int main(int argc, char* argv[])
{
  int iters;
  int path[4096];
  int pathLength;
  vector<POINT> finalpath;

  string PlotDataPass = "./plot_data/";
  string filename;

  if (argc <= 1) {
    cout << "障害物のデータを指定して実行してください。(例: ./rrt ほにゃほにゃ.dat)" << endl;
    return -1;
  } else if (argc == 2) {
    string ObstacleFile = string(argv[1]);
    filename = PlotDataPass + ObstacleFile;
    cout << "ファイルパスは \"" << filename << "\" でし。" << endl;
  } else {
    cout << "引数は1つだけにしてくださいな。" << endl;
    return -1;
  }

  MotionPlan::RRT rrt(filename);
  std::ofstream file("./plot_data/data.dat");


  #ifndef PlotAnimation
  struct timeval start, end;
  gettimeofday(&start, NULL);
  #endif

  rrt.RRTloop(&iters, path, &pathLength, file);
  rrt.OutputFinalPath(&finalpath);
  drowSpline(finalpath);

  #ifndef PlotAnimation
  gettimeofday(&end, NULL);
  cout << "RRTの実行時間は " << (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)*1.0E-6 << "[s]でした。" << endl;
  #endif

  return 0;
}

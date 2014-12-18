#include "RRTStruct.h"

string input_arg(int argc, char* argv[])
{
  string PlotDataPass = "./plot_data/";
  string filename;

  if (argc <= 1) {
    cout << "障害物のデータを指定して実行してください。(例: ./rrt ほにゃほにゃ.dat)" << endl;
    return "ERROR";
  } else if (argc == 2) {
    string ObstacleFile = string(argv[1]);
    filename = PlotDataPass + ObstacleFile;
    cout << "ファイルパスは \"" << filename << "\" でし。" << endl;
    return filename;
  } else {
    cout << "引数は1つだけにしてくださいな。" << endl;
    return "ERROR";
  }
}

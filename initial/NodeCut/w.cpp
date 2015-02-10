#include "function.h"

int main(int argc, char* argv[]){
  /*----------探索をするためのファイルをインプットする----------*/
  string inputfilename[2];
  if(input_arg(argc, argv, inputfilename) == false){
    return -1;
  }

  cout << "区切る距離を教えてください >> ";
  double InputDistance;
  cin >> InputDistance;
  Evaluate ev(inputfilename, InputDistance);
  /*------------------------------------------------------------*/

  // for (unsigned int i = 0; i < data_num; ++i){
  //   cout << pathdata[i].x << ", " << pathdata[i].y << endl;
  // }

  ev.Split();
  ev.Eva();
  ev.output_plt("./data/plot.plt", argv);
  return 0;
}
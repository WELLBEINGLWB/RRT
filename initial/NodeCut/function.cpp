#include "function.h"

bool input_arg(int argc, char* argv[], string filename[])
{
  string RootPass = "./data/";

  if (argc <= 2) {
    cout << "経路のデータを2つ指定して実行してください。(例: ./w 完成した経路データ.dat 障害物データ.dat)" << endl;
    return false;
  } else if (argc == 3) {
    string PathFile = string(argv[1]);
    filename[0] = RootPass + PathFile;
    cout << "経路データのファイルパスは   \"" << filename[0] << "\" でし。" << endl;

    string ObstacleFile = string(argv[2]);
    filename[1] = RootPass + ObstacleFile;
    cout << "障害物データのファイルパスは \"" << filename[1] << "\" でし。" << endl;
    return true;
  } else {
    cout << "引数は2つだけにしてくださいな。" << endl;
    return false;
  }
}



Evaluate::Evaluate(string fileName[], double InputDistance){
  initPathFromFile(fileName[0]);
  cout << "経路の全体の距離は" << PathDistance(data_num) << endl;
  cout << "経路ファイル読み込み完了" << endl;

  initObstacleFromFile(fileName[1]);
  cout << "障害物ファイル読み込み完了" << endl;

  CreatePotentialField();
  cout << "ポテンシャル場の作成完了" << endl;

  inDistance = InputDistance;
  cout << "inDistance = " << inDistance << endl;
}



int Evaluate::CountNumbersOfTextLines(string fileName){
  int i = 0;
  std::ifstream ifs(fileName);         // ファイルを開く。

  if(ifs){                             // ファイルのオープンに成功していれば、これは file は true を返す。
    std::string line;                  // 1行ごとの文字列を格納するための string オブジェクト。
    while(true){
      getline( ifs, line );            // 1行読み取る。
      if( ifs.eof() ){                 // ファイルが終端まで来たら、break 文を実行して while 文を抜ける。
        break;
      }
      i++;                             // 1行読み取ったので、インクリメントする。
    }
  }

  return i;                            // カウントした行数を返す。
}



void Evaluate::initPathFromFile(string fileName)
{
  std::ifstream input(fileName.c_str());
  data_num = CountNumbersOfTextLines(fileName);
  pathdata.resize(data_num);

  for (int i = 0; i < data_num; ++i){
    input >> pathdata[i].x >> pathdata[i].y;
  }

  xStart = pathdata[0].x;
  yStart = pathdata[0].y;
  xGoal = pathdata[data_num-1].x;
  yGoal = pathdata[data_num-1].y;

  cout << data_num << "行の経路ファイルです" << endl;
  input.close();
}



void Evaluate::initObstacleFromFile(string fileName)
{
  std::ifstream input(fileName.c_str());

  input >> xLeft >> xRight >> yBottom >> yTop >> numObstacles;

  xMin = new double[numObstacles];
  xMax = new double[numObstacles];
  yMin = new double[numObstacles];
  yMax = new double[numObstacles];

  for (int i = 0; i < numObstacles; ++i){
    input >> xMin[i] >> xMax[i] >> yMin[i] >> yMax[i];
  }

  input >> xStart >> yStart >> xGoal >> yGoal >> stepSize;

  input.close();

  printf("\nフィールドの定義域は: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf]\n", xLeft, xRight, yBottom, yTop);

  cout << "障害物リスト" << endl;
  for (int i = 0; i < numObstacles; ++i){
    printf("             障害物%d: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf]\n", i+1, xMin[i], xMax[i], yMin[i], yMax[i]);
  }

  printf("\nスタートとゴール    : Start[%5.2lf, %5.2lf]\n", xStart, yStart);
  printf("                        End[%5.2lf, %5.2lf]\n\n", xGoal, yGoal);
}



void Evaluate::CreatePotentialField()
{
  POINT tmp;

  for (int i = 0; i < numObstacles; ++i) {
    for (double x = xMin[i]; x <= xMax[i]; ++x) {
      for(double y = yMin[i]; y <= yMax[i]; ++y){
        tmp.x = x; tmp.y = y;
        obstacle.push_back(tmp);
      }
    }
  }
}


// f(x,y)
double Evaluate::f_xy(double x, double y)
{
  double function;
  double sum = 0.0;

  for (size_t i = 0, size = obstacle.size(); i < size; ++i){
    sum += K*exp(-r_1*pow(x-obstacle[i].x, 2) - r_2*pow(y-obstacle[i].y, 2));
  }

  function = K_1*(pow(x-xGoal, 2) + pow(y-yGoal, 2)) + sum;
  return function;
}



double Evaluate::Distance(POINT a, POINT b)
{
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}



double Evaluate::PathDistance(int max){//maxより小さい要素の距離の総和を出す
  double dis = 0.0;
  if(pathdata.size() == 0){
    std::cout << "パスのデータが存在しません。" << std::endl;
    return -1;
  }else{
    for (int i = 0; i < max-1; ++i) {
      dis += Distance(pathdata[i], pathdata[i+1]);
    }
    // cout << "パスの距離は" << dis << "です。" << endl;
    return dis;
  }
}



void Evaluate::Split()
{
  int Num = PathDistance(data_num)/inDistance;
  int flag = 0;
  double sigma;
  POINT temp;

  for (int i = 1; i <= data_num; ++i){
    D.push_back(PathDistance(i));
  }
  // for(unsigned int i = 0; i < D.size(); i++ ){
  //   cout << D[i] << endl;
  // }

  for (int i = 0; i <= Num; ++i){
    for (unsigned int j = flag; j < D.size(); j++ ){
      if (D[j] <= inDistance*i && inDistance*i < D[j+1]){
        flag = j;
        break;
      }
    }
    sigma = inDistance*i - D[flag];
    temp.x = pathdata[flag].x + sigma*(pathdata[flag+1].x - pathdata[flag].x)/Distance(pathdata[flag+1], pathdata[flag]);
    temp.y = pathdata[flag].y + sigma*(pathdata[flag+1].y - pathdata[flag].y)/Distance(pathdata[flag+1], pathdata[flag]);
    node.push_back(temp);
  }

  std::ofstream pathData("./data/Cut_Node.dat");
  for(unsigned int i = 0; i < node.size(); i++ ){
    pathData << node[i].x << "\t" << node[i].y << endl;
  }
  pathData << xGoal << "\t" << yGoal << endl;
}



void Evaluate::Eva(){
  double MaxCost = 0.0;
  double AveCost = 0.0;
  double SumCost = 0.0;
  double CostCurrent, CostOld, CostDiff;
  double W, Wc = 0.0;
  double S_sum = 0.0, sigma;

  for (unsigned int i = 0; i < node.size(); i++ ){
    CostCurrent = f_xy(node[i].x, node[i].y);
    // コストの合計
    SumCost += CostCurrent;
    // コストの最大値計算
    if(CostCurrent > MaxCost){
      MaxCost = CostCurrent;
    }
    // JailletのW(p)
    if(i > 0){
      CostDiff = CostCurrent - CostOld;
      if(CostDiff <= 0){
        CostDiff = 0;
      }
      Wc += CostDiff * inDistance;
    }
    CostOld = CostCurrent;
  }
  // JailletのW(p)
  W = Wc + 0.01 * stepSize * PathDistance(data_num);

  // コストの平均値
  AveCost = SumCost / node.size();

  // コストの標準偏差
  for (unsigned int i = 0; i < node.size(); i++ ){
    S_sum += pow((f_xy(node[i].x, node[i].y) - AveCost),2);
  }
  sigma = sqrt(S_sum/node.size());

  // 評価データを保存
  time_t t = time(NULL);
  strftime(savefilename, sizeof(savefilename), "./evaluation_data/Evaluate_20%y.%m.%d_%H:%M:%S.dat", localtime(&t));
  std::ofstream data(savefilename);
  data << "Length = " << PathDistance(data_num) << endl;
  data << "MaxCost = " << MaxCost << endl;
  data << "AveCost = " << AveCost << endl;
  data << "SumCost = " << SumCost << endl;
  data << "W(p) = " << W << endl;
  data << "S = " << sigma << endl;
  // data << "Num of Point = " << paths.size() << endl;

  cout << "Length = " << PathDistance(data_num) << endl;
  cout << "MaxCost = " << MaxCost << endl;
  cout << "AveCost = " << AveCost << endl;
  cout << "SumCost = " << SumCost << endl;
  cout << "W(p) = " << W << endl;
  cout << "Wc = " << Wc << endl;
  cout << "S = " << sigma << endl;
  // cout << "Num of Point = " << paths.size() << endl;
}

void Evaluate::CreateCube(string obstacle_output)
{
  RANGE obstacle;
  ofstream cube(obstacle_output);
  for(int ob = 0; ob < numObstacles; ++ob){
    obstacle.xrange[0] = xMin[ob]; obstacle.yrange[0] = yMin[ob];
    obstacle.xrange[1] = xMax[ob]; obstacle.yrange[1] = yMax[ob];
    for (int i = 0; i < 2; ++i){
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << std::endl;
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[0]  << std::endl;
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[1] << std::endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[1] << std::endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << std::endl;
      cube << "\n\n";
    }
  }
}



void Evaluate::output_plt(string plt_output, char* argv[]){
  ofstream plt(plt_output);
  CreateCube("./data/obstacle.dat");

  plt << "set xrange[" << xLeft << ":" << xRight << "]" << endl;
  plt << "set yrange[" << yBottom << ":" << yTop << "]" << endl;
  plt << "set xlabel \"x\"" << endl;
  plt << "set ylabel \"y\"" << endl;

  plt << "set key outside" << endl;
  plt << "set key top right" << endl;

  plt << "set size square" << endl;
  plt << "plot \"Cut_Node.dat\" using 1:2 with points pt 7 ps 0.78 lt rgb \"#ff9900\" title \"cut node\",\\" << endl;
  plt << "\"" << string(argv[1]) << "\"using 1:2 with lines lt 1 lc rgb \"#696969\" lw 1 title \"path\",\\" << endl;
  plt << "\"obstacle.dat\" using 1:2 with filledcurves lt rgb \"#ff0033\" fill solid 0.5 title \'Obstacle\'" << endl;
}

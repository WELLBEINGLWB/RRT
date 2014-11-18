#include "function.h"

using namespace std;

Smoothing::Smoothing(std::string obstaclefile, std::string pathfile)
    : xMin(NULL), xMax(NULL), yMin(NULL), yMax(NULL)
{
  initFromFile(obstaclefile);
  Input_Data(pathfile);
  srand((unsigned int)time(NULL));
}

void Smoothing::initFromFile(std::string fileName)
{
  if (xMin != NULL) delete[] xMin;
  if (xMax != NULL) delete[] xMax;
  if (yMin != NULL) delete[] yMin;
  if (yMax != NULL) delete[] yMax;

  std::ifstream input(fileName.c_str());

  input >> numObstacles;

  xMin = new double[numObstacles];
  xMax = new double[numObstacles];
  yMin = new double[numObstacles];
  yMax = new double[numObstacles];

  for (int i = 0; i < numObstacles; ++i) {
    input >> xMin[i] >> xMax[i] >> yMin[i] >> yMax[i];
  }

  input.close();
}

int Smoothing::CountNumbersOfTextLines(std::string fileName)
{
  int i = 0;
  std::ifstream ifs(fileName);  // ファイルを開く。

  if (ifs) {  // ファイルのオープンに成功していれば、これは file は true を返す。
    std::string line;  // 1行ごとの文字列を格納するための string オブジェクト。
    while (true) {
      getline(ifs, line);  // 1行読み取る。
      if (ifs.eof()) {  // ファイルが終端まで来たら、break 文を実行して while 文を抜ける。
        break;
      }
      i++;  // 1行読み取ったので、インクリメントする。
    }
  }

  return i;  // カウントした行数を返す。
}

void Smoothing::Input_Data(std::string fileName)
{
  int data_num;
  std::ifstream input(fileName.c_str());
  data_num = CountNumbersOfTextLines(fileName);
  paths.resize(data_num);
  cout << "データを読み込み中" << endl;
  for (int i = 0; i < data_num; ++i) {
    input >> paths[i].x >> paths[i].y;
  }

  input.close();
}

double Smoothing::Distance()
{
  double dis = 0.0;
  if(paths.size() == 0){
    cout << "パスのデータが存在しません。" << endl;
    return -1;
  }else{
    for (unsigned int i = 0; i < paths.size()-1; ++i) {
      dis += sqrt(pow(paths[i+1].x - paths[i].x, 2) + pow(paths[i+1].y - paths[i].y, 2));
    }
    // cout << "パスの距離は" << dis << "です。" << endl;
    return dis;
  }
}

bool Smoothing::equal(double x, double y) { return (fabs(x - y) < EPSILON); }

bool Smoothing::clear(const double* xMin, const double* xMax, const double* yMin,
                      const double* yMax, int numObstacles, double xTest, double yTest)
{
  for (int i = 0; i < numObstacles; ++i) {  // 障害物の範囲内ならreturn false
    if (xTest >= xMin[i] && xTest <= xMax[i] && yTest >= yMin[i] && yTest <= yMax[i]) {
      return false;
    }
  }

  return true;  // すべての障害物の中に入ってなかったらreturn true
}

bool Smoothing::link(const double* xMin, const double* xMax, const double* yMin, const double* yMax,
                     int numObstacles, double xStart, double yStart, double xDest, double yDest)
{
  if (!clear(xMin, xMax, yMin, yMax, numObstacles, xStart, yStart) ||
      !clear(xMin, xMax, yMin, yMax, numObstacles, xDest, yDest)) {
    return false;
  }

  double dx = xDest - xStart;
  double dy = yDest - yStart;
  double m = 0.0;
  LineType lineType = GENERAL;

  if (Smoothing::equal(dx, 0))
    lineType = VERTICAL;
  else if (Smoothing::equal(dy, 0))
    lineType = HORIZONTAL;
  else
    m = dy / dx;

  for (int i = 0; i < numObstacles; ++i) {
    if (lineType == VERTICAL) {
      if (xDest >= xMin[i] && xDest <= xMax[i]) {
        if ((yStart >= yMin[i] && yStart <= yMax[i]) || (yDest >= yMin[i] && yDest <= yMax[i]) ||
            (yStart <= yMin[i] && yDest >= yMax[i]) || (yDest <= yMin[i] && yStart >= yMax[i])) {
          return false;
        }
      }
    }

    else if (lineType == HORIZONTAL) {
      if (yDest >= yMin[i] && yDest <= yMax[i]) {
        if ((xStart >= xMin[i] && xStart <= xMax[i]) || (xDest >= xMin[i] && xDest <= xMax[i]) ||
            (xStart <= xMin[i] && xDest >= xMax[i]) || (xDest <= xMin[i] && xStart >= xMax[i])) {
          return false;
        }
      }
    }

    else {  // General line case (slope != 0 and finite)
            // Check for intersection with left side of obstacle
      double y = yStart + m * (xMin[i] - xStart);
      if (y >= yMin[i] && y <= yMax[i]) {
        if ((xMin[i] >= xStart && xMin[i] <= xDest) || (xMin[i] >= xDest && xMin[i] <= xStart)) {
          return false;
        }
      }

      // Check for intersection with right side of obstacle
      y = yStart + m * (xMax[i] - xStart);
      if (y >= yMin[i] && y <= yMax[i]) {
        if ((xMax[i] >= xStart && xMax[i] <= xDest) || (xMax[i] >= xDest && xMax[i] <= xStart)) {
          return false;
        }
      }

      // Check for intersection with bottom of obstacle
      double x = (yMin[i] - yStart) / m + xStart;
      if (x >= xMin[i] && x <= xMax[i]) {
        if ((yMin[i] >= yStart && yMin[i] <= yDest) || (yMin[i] >= yDest && yMin[i] <= yStart)) {
          return false;
        }
      }

      // Check for intersection with top of obstacle
      x = (yMax[i] - yStart) / m + xStart;
      if (x >= xMin[i] && x <= xMax[i]) {
        if ((yMax[i] >= yStart && yMax[i] <= yDest) || (yMax[i] >= yDest && yMax[i] <= yStart)) {
          return false;
        }
      }
    }
  }

  return true;
}

int Smoothing::GetRandom(double min, double max)
{
  int R;

  R = min + (int)(rand() * ((max - min) + 1.0) / (1.0 + RAND_MAX));
  return R;
}

void Smoothing::PrintData()
{
  // for (unsigned int i = 0; i < paths.size(); i++) {
  //   cout << "( " << i << ", " << paths[i].x << ", " << paths[i].y << " )" << endl;
  // }
  for (unsigned int i = 0; i < paths.size(); i++) {
    cout << paths[i].x << "\t" << paths[i].y << endl;
  }
}

void Smoothing::OutputData()
{
  ofstream output("./plot_data/path_data_mod.dat");

  for (unsigned int i = 0; i < paths.size(); i++) {
    output << paths[i].x << "\t" << paths[i].y << endl;
  }
}

void Smoothing::PrintObstacle()
{
  for (int i = 0; i < numObstacles; i++) {
    cout << "障害物" << i+1 << " ( xMin[" << i << "] = " << xMin[i] << ", xMax[" << i << "] = " << xMax[i] <<
    "\n          yMin[" << i << "] = " << yMin[i] << ", yMax[" << i << "] = " << yMax[i] << " )" << endl;
  }
}

void Smoothing::smoothing(int loop)
{
  int SamplePoint[2];
  int tmp;
  double ini ,current, old = 0.0;

  ini = Distance();
  cout << "初期のパスの距離は" << ini << endl;
  for (int i = 0; i < loop; ++i){
    while (1) {
      for (int j = 0; j < 2; ++j) {
        SamplePoint[j] = Smoothing::GetRandom(0, paths.size() - 1);
      }
      if (SamplePoint[0] > SamplePoint[1] && SamplePoint[0] != SamplePoint[1] + 1) {
        tmp = SamplePoint[0];
        SamplePoint[0] = SamplePoint[1];
        SamplePoint[1] = tmp;
        break;
      } else if (SamplePoint[0] < SamplePoint[1] && SamplePoint[0] + 1 != SamplePoint[1]) {
        break;
      } else {
        //もう一回引き直し(ΦωΦ)
      }
    }

    if (link(xMin, xMax, yMin, yMax, numObstacles,
             paths[SamplePoint[0]].x, paths[SamplePoint[0]].y,
             paths[SamplePoint[1]].x, paths[SamplePoint[1]].y)){
      cout << SamplePoint[0] << "と" << SamplePoint[1] << "の間の点はグッバイ！" << endl;
      cout << "グッバイしたあとのpathの長さは" << paths.size() << "です。" << endl;
      paths.erase(paths.begin()+SamplePoint[0]+1, paths.begin()+SamplePoint[1]);

      std::ofstream outStream("./plot_data/path_data_mod.dat", std::ios_base::trunc);
      for (unsigned int k = 0; k < paths.size(); k++) {
        outStream << paths[k].x << "\t" << paths[k].y << endl;
      }
      for (int k = 0; k < 100; ++k){
        usleep(10000);
      }


      current = Distance();
      cout << "現在のパスの総距離は" << current << endl;
      // if( (ini - current) > 7){
      //   cout << "しきい値以下になりました！" << endl;
      //   break;
      // }
      if(fabs(old - current) < 0.0000001){
        cout << "しきい値以下になりました！" << endl;
        break;
      }
      old = current;
    }

  }
  //PrintData();
}

void Smoothing::onestep_smoothing(int loop)
{
  int SamplePoint[2];

  for (int i = 0; i < loop; ++i){
    SamplePoint[0] = Smoothing::GetRandom(0, paths.size() - 2);
    SamplePoint[1] = SamplePoint[0] + 2;

    if (link(xMin, xMax, yMin, yMax, numObstacles,
             paths[SamplePoint[0]].x, paths[SamplePoint[0]].y,
             paths[SamplePoint[1]].x, paths[SamplePoint[1]].y)){
      cout << SamplePoint[0] << "と" << SamplePoint[1] << "の間の点はグッバイ！" << endl;
      cout << "グッバイしたあとのpathの長さは" << paths.size() << "です。" << endl;
      paths.erase(paths.begin()+SamplePoint[0]+1);
    }
  }
  PrintData();
}
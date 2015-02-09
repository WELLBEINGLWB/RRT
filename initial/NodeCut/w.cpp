#include <iostream>
#include <fstream>
#include <ostream>

#include <iterator>
#include <vector>
#include <string>
#include <algorithm>

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <sys/time.h>
#include <unistd.h>

using namespace std;

typedef struct{
  double x;
  double y;
} POINT;

int data_num;
vector< POINT > pathdata, node;

double StartP[2], GoalP[2];

void Split(double length);
double Distance(POINT a, POINT b);
double PathDistance(int max);
void input(string fileName);
int CountNumbersOfTextLines(string fileName);
void Input_Data(string fileName);

int main(void){
  Input_Data("./path.dat");
  cout << "経路の全体の距離は" << PathDistance(data_num) << endl;
  for (unsigned int i = 0; i < data_num; ++i){
    cout << pathdata[i].x << ", " << pathdata[i].y << endl;
  }

  cout << "区切る距離を教えてください >> ";
  double input;
  cin >> input;
  Split(input);
}


void Split(double length)
{
  int Num = PathDistance(data_num)/length;
  vector< double > D;
  int flag = 0;
  double sigma;
  POINT temp;

  for (int i = 1; i <= data_num; ++i){
    D.push_back(PathDistance(i));
  }
  for(unsigned int i = 0; i < D.size(); i++ ){
    cout << D[i] << endl;
  }

  for (int i = 0; i <= Num; ++i){
    for (unsigned int j = flag; j < D.size(); j++ ){
      if (D[j] <= length*i && length*i < D[j+1]){
        flag = j;
        break;
      }
    }
    sigma = length*i - D[flag];
    temp.x = pathdata[flag].x + sigma*(pathdata[flag+1].x - pathdata[flag].x)/Distance(pathdata[flag+1], pathdata[flag]);
    temp.y = pathdata[flag].y + sigma*(pathdata[flag+1].y - pathdata[flag].y)/Distance(pathdata[flag+1], pathdata[flag]);
    node.push_back(temp);
  }

  std::ofstream pathData("./path_data.dat");
  for(unsigned int i = 0; i < node.size(); i++ ){
    pathData << node[i].x << "\t" << node[i].y << endl;
  }
  pathData << GoalP[0] << "\t" << GoalP[1] << endl;
}


double PathDistance(int max){//maxより小さい要素の距離の総和を出す
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


double Distance(POINT a, POINT b)
{
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}


int CountNumbersOfTextLines(string fileName){
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


void Input_Data(string fileName)
{
  std::ifstream input(fileName.c_str());
  data_num = CountNumbersOfTextLines(fileName);
  pathdata.resize(data_num);

  for (int i = 0; i < data_num; ++i){
    input >> pathdata[i].x >> pathdata[i].y;
  }

  StartP[0] = pathdata[0].x;
  StartP[1] = pathdata[0].y;
  GoalP[0] = pathdata[data_num-1].x;
  GoalP[1] = pathdata[data_num-1].y;

  cout << data_num << "行のファイル読み込み成功" << endl;
  input.close();
}

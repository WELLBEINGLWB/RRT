#include "APFChecker.h"

APF::APF(POINT s, POINT g, string fileName){
  start.x = s.x; start.y = s.y; start.z = s.z;
  goal.x = g.x; goal.y = g.y; goal.z = g.z;
  Input_Data(fileName);
  InitialCondition();
}


APF::APF(POINT s, POINT g, string fileName, RANGE o){
  start.x = s.x; start.y = s.y; start.z = s.z;
  goal.x = g.x; goal.y = g.y; goal.z = g.z;

  for (int i = 0; i < 2; ++i){
    obstacle.xrange[i] = o.xrange[i];
    obstacle.yrange[i] = o.yrange[i];
    obstacle.zrange[i] = o.zrange[i];
  }

  CreateCube(obstacle, fileName);
  Input_Data(fileName);
  InitialCondition();
}


void APF::InitialCondition(){
  cout << "コンストラクタ呼ばれました！" << endl;
  cout << "初期値(スタート)は" << endl;
  cout << "x = " << start.x << endl;
  cout << "y = " << start.y << endl;
  cout << "z = " << start.z << endl;
  cout << "極値(ゴール)は" << endl;
  cout << "x = " << goal.x << endl;
  cout << "y = " << goal.y << endl;
  cout << "z = " << goal.z << endl;
}


int APF::CountNumbersOfTextLines(string fileName){
  int i = 0;
  ifstream ifs(fileName);// ファイルを開く。

  if(ifs){// ファイルのオープンに成功していれば、これは file は true を返す。
    string line;// 1行ごとの文字列を格納するための string オブジェクト。
    while( true ){
      getline( ifs, line );// 1行読み取る。
      if( ifs.eof() ){// ファイルが終端まで来たら、break 文を実行して while 文を抜ける。
        break;
      }
      i++;// 1行読み取ったので、インクリメントする。
    }
  }

  return i;// カウントした行数を返す。
}


void APF::Input_Data(string fileName){
  int num;
  //POINT pt;

  std::ifstream input(fileName.c_str());
  num = CountNumbersOfTextLines(fileName);
  vobstacle.resize(num);

  for (int i = 0; i < num; ++i){
    input >> vobstacle[i].x >> vobstacle[i].y >> vobstacle[i].z;
  }

  input.close();
}


void APF::CreateCube(RANGE obstacle, string fileName){
  ofstream out_put_data(fileName);

  for(int x = obstacle.xrange[0]; x <= obstacle.xrange[1]; ++x) {
    for(int y = obstacle.yrange[0]; y <= obstacle.yrange[1]; ++y){
      for(int z = obstacle.zrange[0]; z <= obstacle.zrange[1]; ++z){
        out_put_data << x << " " << y << " " << z << endl;
      }
    }
  }
}


// f(x,y)
double APF::f_xyz(double x,double y, double z){
  double function;
  double sum = 0.0;

  for (size_t i = 0, size = vobstacle.size(); i < size; ++i){
    sum += K*exp(-r_1*pow(x-vobstacle[i].x, 2) - r_2*pow(y-vobstacle[i].y, 2) - r_3*pow(z-vobstacle[i].z, 2));
  }

  function = K_1*(pow(x-goal.x, 2) + pow(y-goal.y, 2) + pow(z-goal.z, 2)) + sum;
  return function;
}


// df/dx
double APF::fx(double x, double y, double z){
  double function;
  double sum = 0.0;

  for (size_t i = 0, size = vobstacle.size(); i < size; ++i){
    sum += 2*K*r_1*(x-vobstacle[i].x)*exp(-r_1*pow(x-vobstacle[i].x, 2) - r_2*pow(y-vobstacle[i].y, 2) - r_3*pow(z-vobstacle[i].z, 2));
  }

  function = 2*K_1*(x-goal.x) - sum;
  return function;
}


// df/dy
double APF::fy(double x,double y, double z){
  double function;
  double sum = 0.0;

  for (size_t i = 0, size = vobstacle.size(); i < size; ++i){
    sum += 2*K*r_2*(y - vobstacle[i].y)*exp(-r_1*pow(x-vobstacle[i].x, 2) - r_2*pow(y-vobstacle[i].y, 2) - r_3*pow(z-vobstacle[i].z, 2));
  }

  function = 2*K_1*(y-goal.y) - sum;
  return function;
}


// df/dy
double APF::fz(double x,double y, double z){
  double function;
  double sum = 0.0;

  for (size_t i = 0, size = vobstacle.size(); i < size; ++i){
    sum += 2*K*r_3*(z - vobstacle[i].z)*exp(-r_1*pow(x-vobstacle[i].x, 2) - r_2*pow(y-vobstacle[i].y, 2) - r_3*pow(z-vobstacle[i].z, 2));
  }

  function = 2*K_1*(z-goal.z) - sum;
  return function;
}

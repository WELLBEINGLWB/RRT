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


// void APF::gridf(double xs[],double ys[],double zs[]){
//  int i,j;                                // 格子点インデクス
//  double x,y,z,fxyz;                      // 格子点座標、関数値

//  cout << "distribution over 0<=x,y<=" << MAX_FIELD << endl;
//  cout << "preliminary research of f(x,y)" << endl;
//  cout << "f(x,y)" << endl;

//  for(i=0;i<=MAX_FIELD;i++) {
//    for(j=0;j<=MAX_FIELD;j++) {
//      x = xs[0]+j*(xs[1]-xs[0])/MAX_FIELD;
//      y = ys[0]+i*(ys[1]-ys[0])/MAX_FIELD;
//      z = zs[0]+i*(zs[1]-zs[0])/MAX_FIELD;
//      fxyz = f_xyz(x,y,z);
//      printf(" %lf",fxyz);
//    } printf("\n");
//  }

//  cout << "df(x,y)/dx" << endl;
//  for(i=0;i<=MAX_FIELD;i++) {
//    for(j=0;j<=MAX_FIELD;j++) {
//      x = xs[0]+j*(xs[1]-xs[0])/MAX_FIELD;
//      y = ys[0]+i*(ys[1]-ys[0])/MAX_FIELD;
//      z = zs[0]+i*(zs[1]-zs[0])/MAX_FIELD;
//      fxyz = fx(x,y,z);
//      printf(" %lf",fxyz);
//    } printf("\n");
//  }

//  cout << "df(x,y)/dy" << endl;
//  for(i=0;i<=MAX_FIELD;i++) {
//    for(j=0;j<=MAX_FIELD;j++) {
//      x = xs[0]+j*(xs[1]-xs[0])/MAX_FIELD;
//      y = ys[0]+i*(ys[1]-ys[0])/MAX_FIELD;
//      z = zs[0]+i*(zs[1]-zs[0])/MAX_FIELD;
//      fxyz = fy(x,y,z);
//      printf(" %lf",fxyz);
//    } printf("\n");
//  }

//  cout << "df(x,y)/dz" << endl;
//  for(i=0;i<=MAX_FIELD;i++) {
//    for(j=0;j<=MAX_FIELD;j++) {
//      x = xs[0]+j*(xs[1]-xs[0])/MAX_FIELD;
//      y = ys[0]+i*(ys[1]-ys[0])/MAX_FIELD;
//      z = zs[0]+i*(zs[1]-zs[0])/MAX_FIELD;
//      fxyz = fz(x,y,z);
//      printf(" %lf",fxyz);
//    } printf("\n");
//  }

// }


POINT APF::extreme(double xs[], double ys[], double zs[], string dat_output){
  double x,y,z;         // 座標
  double h,k,l;         // 勾配
  double alf=0.001;   // α係数
  double dx,dy,dz;        // 座標修正量
  //double err;         // 勾配の絶対値
  double eps=0.00001; // 許容誤差
  POINT p;            // 計算点
  int n=0;            // 繰返し回数
  int m=1000000;      // 繰返し回数制限

  POINT p_tmp, dp_tmp;

  vector< POINT > vec_p, vec_dp;

  ofstream out_put_data(dat_output);

  cout << "execute Steepest descent method" << endl;
  cout << "alf = " << alf << " eps = " << eps << endl;
  x = start.x; y = start.y; z = start.z;
  do {
    n++;

    h = fx(x,y,z); k = fy(x,y,z); l = fz(x,y,z);
    dx = alf*h; dy = alf*k; dz = alf*l;
    x -= dx; y -= dy; z -= dz;

    // 領域境界制限（追加）
    if(x<xs[0]) x = xs[0];
    if(x>xs[1]) x = xs[1];
    if(y<ys[0]) y = ys[0];
    if(y>ys[1]) y = ys[1];
    if(z<zs[0]) z = zs[0];
    if(z>zs[1]) z = zs[1];

    p_tmp.x = x; p_tmp.y = y; p_tmp.z = z;
    dp_tmp.x = h; dp_tmp.y = k; dp_tmp.z = l;

    vec_p.push_back(p_tmp);
    vec_dp.push_back(dp_tmp);

    //err = fabs(h) + fabs(k);
    if(n>m) {
      cout << n << "iteration, abort." << endl;
      break;
    }
  } while(!(-eps < h && h<eps && -eps < k && k< eps && -eps < l && l<eps));

  // for(size_t i = 0, size = vec_p.size(); i < size; ++i) {
  //  out_put_data << vec_p[i].x << " " <<vec_p[i].y << " " << vec_p[i].z << " " << vec_dp[i].x << " " << vec_dp[i].y << " " << vec_dp[i].z << endl;
  // }
  for(size_t i = 0, size = vec_p.size(); i < size; ++i) {
    out_put_data << vec_p[i].x/10.0 << " " <<vec_p[i].y/10.0 << " " << vec_p[i].z/10.0 << endl;
  }

  cout << "finished after " << n << " iterations" << endl;
  p.x = x; p.y = y; p.z = z;

  return p;
}


void APF::output_result(POINT p){

  cout << "ゴールした時の座標は: " << endl;
  cout << "x = " << p.x << endl;
  cout << "y = " << p.y << endl;
  cout << "z = " << p.z << endl;
  cout << "そのときのそれぞれの値は: " << endl;
  cout << "f = " << fz(p.x,p.y,p.z) << endl;
  cout << "fx = " << fx(p.x,p.y,p.z) << endl;
  cout << "fy = " << fy(p.x,p.y,p.z) << endl;
  }


void APF::output_plt(string plt_output){
  ofstream plt(plt_output);

  plt << "set xlabel \"x\""<< endl;
  plt << "set ylabel \"y\"" << endl;
  plt << "set zlabel \"z\"" << endl;
  plt << "set xrange [" << -(MAX_FIELD+1) << ":" << MAX_FIELD+1 << "]" << endl;
  plt << "set yrange [" << -(MAX_FIELD+1) << ":" << MAX_FIELD+1 << "]" << endl;
  plt << "set zrange [" << 0 << ":" << MAX_FIELD+1 << "]" << endl;

  plt << "splot \"input.dat\" using 1:2:3 with points pt 7 ps 2 lt rgb \"#696969\" title \"Obstacle\"" << endl;
  plt << "replot \"data.dat\" using 1:2:3 with points pt 7 ps 1 lt rgb \"#FF5E19\" title \"Path\"" << endl;

}

// void APF::output_plt(string plt_output, string cube_output){
//  ofstream plt(plt_output);
//  ofstream cube(cube_output);

//  plt << "set xlabel \"x\""<< endl;
//  plt << "set ylabel \"y\"" << endl;
//  plt << "set zlabel \"z\"" << endl;
//  plt << "set xrange [" << -(MAX_FIELD+1) << ":" << MAX_FIELD+1 << "]" << endl;
//  plt << "set yrange [" << -(MAX_FIELD+1) << ":" << MAX_FIELD+1 << "]" << endl;
//  plt << "set zrange [" << 0 << ":" << MAX_FIELD+1 << "]" << endl;
//  plt << "set ticslevel 0\n" << endl;

//  plt << "splot \"cube.dat\" using 1:2:3 with lines lw 5 lt rgb \"#696969\" title \"Obstacle\"" << endl;
//  //plt << "splot \"cube.dat\" using 1:2:3 lw 5 lt rgb \"#696969\" title \"Obstacle\"" << endl;
//  plt << "replot \"data.dat\" using 1:2:3 with points pt 7 ps 1 lt rgb \"#FF5E19\" title \"Path\"" << endl;

//  for (int i = 0; i < 2; ++i){
//    cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << endl;
//    cube << obstacle.xrange[1] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << endl;
//    cube << obstacle.xrange[1] << "\t" << obstacle.yrange[1] << "\t" << obstacle.zrange[i] << endl;
//    cube << obstacle.xrange[0] << "\t" << obstacle.yrange[1] << "\t" << obstacle.zrange[i] << endl;
//    cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << endl;
//    cube << "\n\n";
//  }

//  for (int i = 0; i < 2; ++i){
//    for (int j = 0; j < 2; ++j){
//      for (int k = 0; k < 2; ++k){
//        cube << obstacle.xrange[i] << "\t" << obstacle.yrange[j] << "\t" << obstacle.zrange[k] << endl;
//      }
//      cube << "\n\n";
//    }
//  }
// }

void APF::output_plt(string plt_output, string cube_output){
  ofstream plt(plt_output);
  ofstream cube(cube_output);

  plt << "set xlabel \"x\""<< endl;
  plt << "set ylabel \"y\"" << endl;
  plt << "set zlabel \"z\"" << endl;
  plt << "set xrange [" << -(MAX_FIELD+1) << ":" << MAX_FIELD+1 << "]" << endl;
  plt << "set yrange [" << -(MAX_FIELD+1) << ":" << MAX_FIELD+1 << "]" << endl;
  plt << "set zrange [" << 0 << ":" << MAX_FIELD+1 << "]" << endl;
  plt << "set ticslevel 0\n" << endl;

  plt << "splot \"cube.dat\" using 1:2:3 with lines lw 5 lt rgb \"#696969\" title \"Obstacle\"" << endl;
  //plt << "splot \"cube.dat\" using 1:2:3 lw 5 lt rgb \"#696969\" title \"Obstacle\"" << endl;
  plt << "replot \"data.dat\" using 1:2:3 with points pt 7 ps 1 lt rgb \"#FF5E19\" title \"Path\"" << endl;

  for (int i = 0; i < 2; ++i){
    cube << obstacle.xrange[0]/10.0 << "\t" << obstacle.yrange[0]/10.0 << "\t" << obstacle.zrange[i]/10.0 << endl;
    cube << obstacle.xrange[1]/10.0 << "\t" << obstacle.yrange[0]/10.0 << "\t" << obstacle.zrange[i]/10.0 << endl;
    cube << obstacle.xrange[1]/10.0 << "\t" << obstacle.yrange[1]/10.0 << "\t" << obstacle.zrange[i]/10.0 << endl;
    cube << obstacle.xrange[0]/10.0 << "\t" << obstacle.yrange[1]/10.0 << "\t" << obstacle.zrange[i]/10.0 << endl;
    cube << obstacle.xrange[0]/10.0 << "\t" << obstacle.yrange[0]/10.0 << "\t" << obstacle.zrange[i]/10.0 << endl;
    cube << "\n\n";
  }

  for (int i = 0; i < 2; ++i){
    for (int j = 0; j < 2; ++j){
      for (int k = 0; k < 2; ++k){
        cube << obstacle.xrange[i]/10.0 << "\t" << obstacle.yrange[j]/10.0 << "\t" << obstacle.zrange[k]/10.0 << endl;
      }
      cube << "\n\n";
    }
  }
}
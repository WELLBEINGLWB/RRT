#include "Bspline.h"

//スプラインデータ初期化
void Draw::Spline::init(double *sp, int spnum)
{
  double tmp, w[MaxSplineSize + 1];
  int i;

  num = spnum - 1;

  // ３次多項式の0次係数(a)を設定
  for (i = 0; i <= num; i++) {
    a[i] = sp[i];
  }

  // ３次多項式の2次係数(c)を計算
  // 連立方程式を解く。
  // 但し、一般解法でなくスプライン計算にチューニングした方法
  c[0] = c[num] = 0.0;
  for (i = 1; i < num; i++) {
    c[i] = 3.0 * (a[i - 1] - 2.0 * a[i] + a[i + 1]);
  }
  // 左下を消す
  w[0] = 0.0;
  for (i = 1; i < num; i++) {
    tmp = 4.0 - w[i - 1];
    c[i] = (c[i] - c[i - 1]) / tmp;
    w[i] = 1.0 / tmp;
  }
  // 右上を消す
  for (i = num - 1; i > 0; i--) {
    c[i] = c[i] - c[i + 1] * w[i];
  }

  // ３次多項式の1次係数(b)と3次係数(b)を計算
  b[num] = d[num] = 0.0;
  for (i = 0; i < num; i++) {
    d[i] = (c[i + 1] - c[i]) / 3.0;
    b[i] = a[i + 1] - a[i] - c[i] - d[i];
  }
}


//媒介変数(0～num-1の実数）に対する値を計算
double Draw::Spline::calc(double t)
{
  int j;
  double dt;

  j = (int)floor(t); // 小数点以下切捨て
  if(j < 0){
    j = 0;
  } else if (j >= num){
    j = num - 1; // 丸め誤差を考慮
  }

  dt = t - (double)j;
  return a[j] + ( b[j] + (c[j] + d[j] * dt) * dt ) * dt;
}


Draw::Draw(MotionPlan::RRT &rrt)
{
  CreatePotentialField(rrt);
}


void Draw::CreatePotentialField(MotionPlan::RRT &rrt)
{
  POINT tmp;

  cout << "ポテンシャル場の作成" << endl;
  for (int i = 0; i < rrt.numObstacles; ++i){
    for(double x = rrt.xMin[i]; x <= rrt.xMax[i]; ++x) {
      for(double y = rrt.yMin[i]; y <= rrt.yMax[i]; ++y){
        for(double z = rrt.zMin[i]; z <= rrt.zMax[i];  ++z){
          tmp.x = x; tmp.y = y; tmp.z = z;
          vobstacle.push_back(tmp);
        }
      }
    }
  }
}


// f(x,y)
double Draw::f_xyz(double x,double y, double z)
{
  double sum = 0.0;

  for (size_t i = 0, size = vobstacle.size(); i < size; ++i){
    sum += K*exp(-r_1*pow(x-vobstacle[i].x, 2) - r_2*pow(y-vobstacle[i].y, 2) - r_3*pow(z-vobstacle[i].z, 2));
  }

  return sum;
}


// Ｂスプライン描画
// x[num], y[num], z[num] は座標の配列
void Draw::drowSpline(std::vector<POINT> &finalpath)
{
  double Potential = 0.0;
  double PotentialSum = 0.0;
  double MaxPotential = 0.0;
  double t, m;
  POINT tmp;
  POINT MaxPoint;
  Spline xs, ys, zs;

  num = finalpath.size();
  x = new double[num];
  y = new double[num];
  z = new double[num];

  for (int i = 0; i < num; ++i) {
    x[i] = finalpath[i].x;
    y[i] = finalpath[i].y;
    z[i] = finalpath[i].z;
  }

  xs.init(x, num);
  ys.init(y, num);
  zs.init(z, num);

  m = (double)(num - 1);
  std::ofstream outStream("./plot_data/Bspline.dat");
  // std::ofstream outdata("./plot_data/Potential.dat");

  for (t = 0; t <= m; t += 0.01) {
    tmp.x = xs.calc(t); tmp.y = ys.calc(t); tmp.z = zs.calc(t);
    Potential = f_xyz(tmp.x, tmp.y, tmp.z);  // その座標のポテンシャル計算
    PotentialSum += Potential;               // ポテンシャルの合計を計算

    if (Potential > MaxPotential) {  // 経路中でもっとも高いポテンシャルを計算（最大値計算）
      MaxPotential = Potential;
      MaxPoint.x = tmp.x; MaxPoint.y = tmp.y; MaxPoint.z = tmp.z;
    }

    SplinePoint.push_back(tmp);
    outStream << tmp.x << "\t" << tmp.y << "\t" << tmp.z << std::endl;
    // outdata << tmp.x << "\t" << tmp.y << "\t" << tmp.z << "\t" << f_xyz(tmp.x, tmp.y, tmp.z) << std::endl;

    #ifdef PlotAnimation
    for (int k = 0; k < 100; ++k){
      usleep(50);
    }
    #endif
  }
  cout << "経路中の最も高いポテンシャル = " << MaxPotential << endl;
  cout << "そのときの座標は = (" << MaxPoint.x << ", " << MaxPoint.y << ", " << MaxPoint.z << ")" << endl;
  cout << "平滑化後の経路のポテンシャルの合計 = " << PotentialSum << endl;

}
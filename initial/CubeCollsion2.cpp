#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>

using namespace std;

typedef struct{
  double x;
  double y;
  double z;
} POINT;

typedef struct {
  double xrange[2];
  double yrange[2];
  double zrange[2];
} RANGE;


void CreateGridPoint(const double* xMin, const double* xMax,
                     const double* yMin, const double* yMax,
                     const double* zMin, const double* zMax, POINT *P, int i){

  P[0].x = xMin[i]; P[0].y = yMin[i]; P[0].z = zMin[i];
  P[1].x = xMax[i]; P[1].y = yMin[i]; P[1].z = zMin[i];
  P[2].x = xMax[i]; P[2].y = yMax[i]; P[2].z = zMin[i];
  P[3].x = xMin[i]; P[3].y = yMax[i]; P[3].z = zMin[i];
  P[4].x = xMin[i]; P[4].y = yMin[i]; P[4].z = zMax[i];
  P[5].x = xMax[i]; P[5].y = yMin[i]; P[5].z = zMax[i];
  P[6].x = xMax[i]; P[6].y = yMax[i]; P[6].z = zMax[i];
  P[7].x = xMin[i]; P[7].y = yMax[i]; P[7].z = zMax[i];

}


void PlaneEquation(POINT p[], int i0[], int i1[], int i2[], int i, double a[]){

  a[0] = (p[i1[i]].y-p[i0[i]].y)*(p[i2[i]].z-p[i0[i]].z)-(p[i2[i]].y-p[i0[i]].y)*(p[i1[i]].z-p[i0[i]].z);
  a[1] = (p[i1[i]].z-p[i0[i]].z)*(p[i2[i]].x-p[i0[i]].x)-(p[i2[i]].z-p[i0[i]].z)*(p[i1[i]].x-p[i0[i]].x);
  a[2] = (p[i1[i]].x-p[i0[i]].x)*(p[i2[i]].y-p[i0[i]].y)-(p[i2[i]].x-p[i0[i]].x)*(p[i1[i]].y-p[i0[i]].y);
  a[3] = a[0]*p[i0[i]].x + a[1]*p[i0[i]].y + a[2]*p[i0[i]].z;

  //cout << a[0] << " x + " << a[1] << " y + " << a[2] <<  " z + " << a[3] << " = 0" << endl;
}


void Pcompare(POINT A, POINT B, POINT *compare){

  if ((A.x - B.x) > 0) {
    compare[0].x = B.x;
    compare[1].x = A.x;
  }else{
    compare[0].x = A.x;
    compare[1].x = B.x;
  }

  if((A.y - B.y) > 0) {
    compare[0].y = B.y;
    compare[1].y = A.y;
  }else{
    compare[0].y = A.y;
    compare[1].y = B.y;
  }

  if((A.z - B.z) > 0) {
    compare[0].z = B.z;
    compare[1].z = A.z;
  }else{
    compare[0].z = A.z;
    compare[1].z = B.z;
  }
}


bool Link(const double* xMin, const double* xMax,
                  const double* yMin, const double* yMax,
                  const double* zMin, const double* zMax,
                  int numObstacles,
                  double xStart, double yStart, double zStart,
                  double xDest, double yDest, double zDest){

  int i = 0;
  bool flag = true;
  POINT A, B, P, p[8], E, compare[2];
  double a[4];
  double t, DE;
  int i0[6] = {0,0,1,2,3,4};
  int i1[6] = {1,1,2,3,4,5};
  int i2[6] = {2,4,5,6,7,6};

  A.x = xStart; A.y = yStart; A.z = zStart;
  B.x = xDest; B.y = yDest; B.z = zDest;
  E.x = A.x - B.x;
  E.y = A.y - B.y;
  E.z = A.z - B.z;
  Pcompare(A, B, compare);

  for (int j = 0; j < numObstacles; ++j) {
    CreateGridPoint(xMin, xMax, yMin, yMax, zMin, zMax, p, j);

    do{
      PlaneEquation(p, i0, i1, i2, i, a);

      t = (a[3] - (a[0] * A.x + a[1] * A.y + a[2] * A.z)) /
          (a[0] * E.x + a[1] * E.y + a[2] * E.z);
      DE = a[0] * E.x + a[1] * E.y + a[2] * E.z;

      if(DE == 0){
        cout << j+1 << "番目の障害物: " << i << "面と直線との交点なし" << endl;
        //flag = true;
      }else{
        // cout << "交点あり" << endl;
        P.x = A.x + t * E.x;
        P.y = A.y + t * E.y;
        P.z = A.z + t * E.z;
        cout << j+1 << "番目の障害物: " << i << "面, (" << P.x << ", " << P.y << ", " << P.z << ")" << endl;

        // if(i==5){
        //   cout << compare[0].x << "<= x <="<< compare[1].x << endl;
        //   cout << compare[0].y << "<= y <="<< compare[1].y << endl;
        //   cout << compare[0].z << "<= z <="<< compare[1].z << endl;
        //   cout << xMin[i] << "<= P.x <=" << xMax[i] << endl;
        //   cout << yMin[i] << "<= P.y <=" << yMax[i] << endl;
        //   cout << zMin[i] << "<= P.z <=" << zMax[i] << endl;
        // }

        if (xMin[j] <= P.x && P.x <= xMax[j] &&
            yMin[j] <= P.y && P.y <= yMax[j] &&
            zMin[j] <= P.z && P.z <= zMax[j] &&
            compare[0].x <= P.x && P.x <= compare[1].x &&
            compare[0].y <= P.y && P.y <= compare[1].y &&
            compare[0].z <= P.z && P.z <= compare[1].z) {
          cout << j+1 << "番目の障害物: " << i << "面, 定義した4点内に入っている" << endl;
          flag = false;
          break;
        }else{
          cout << j+1 << "番目の障害物: " << i << "面, 範囲外" << endl;
          //flag = true;
        }
      }
      i++;
    }while(i<6);
    i = 0;
  }

  return flag;
}


void CreateCube(const double* xMin, const double* xMax, const double* yMin,
                const double* yMax, const double* zMin, const double* zMax,
                int numObstacles,
                double xStart, double yStart, double zStart,
                double xDest, double yDest, double zDest,
                string fileName)
{
  ofstream cube(fileName);

  RANGE obstacle;

  for(int ob = 0; ob < numObstacles; ++ob){
    obstacle.xrange[0] = xMin[ob]; obstacle.yrange[0] = yMin[ob]; obstacle.zrange[0] = zMin[ob];
    obstacle.xrange[1] = xMax[ob]; obstacle.yrange[1] = yMax[ob]; obstacle.zrange[1] = zMax[ob];
    for (int i = 0; i < 2; ++i){
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << endl;
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << endl;
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[1] << "\t" << obstacle.zrange[i] << endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[1] << "\t" << obstacle.zrange[i] << endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << endl;
      cube << "\n\n";
    }

    for (int i = 0; i < 2; ++i){
      for (int j = 0; j < 2; ++j){
        for (int k = 0; k < 2; ++k){
          cube << obstacle.xrange[i] << "\t" << obstacle.yrange[j] << "\t" << obstacle.zrange[k] << endl;
        }
        cube << "\n\n";
      }
    }
  }

  cube << xStart << "\t" << yStart << "\t" << zStart << endl;
  cube << xDest << "\t" << yDest << "\t" << zDest << endl;

}

void output_plt(string plt_output){
  ofstream plt(plt_output);

  plt << "set xlabel \"x\""<< endl;
  plt << "set ylabel \"y\"" << endl;
  plt << "set zlabel \"z\"" << endl;
  plt << "set ticslevel 0" << endl;
  plt << "splot \"cube.dat\" using 1:2:3 with lines lw 5 lt rgb \"#696969\" title \"Obstacle\"" << endl;
}


int main(){
  // POINT start = {0,2,1.5};
  // POINT end = {4,2,1.5};;
  bool output;
  double xMin[3]={1,2,3}, xMax[3]={1.8,2.8,3.8};
  double yMin[3]={1,2,3}, yMax[3]={1.8,2.8,3.8};
  double zMin[3]={1,2,3}, zMax[3]={1.8,2.8,3.8};

  double xStart = 3, yStart = 1, zStart =1;
  double xDest = 4, yDest = 4, zDest = 4;

  output = Link(xMin, xMax, yMin, yMax, zMin, zMax,
                        3,
                        xStart, yStart, zStart,
                        xDest, yDest, zDest);
  CreateCube(xMin, xMax, yMin,
             yMax, zMin, zMax,
             3,
             xStart, yStart, zStart,
             xDest, yDest, zDest,
             "cube.dat");
  output_plt("output.plt");

  if(output == true){
    cout << "当たってない!" << endl;
  }else{
    cout << "当たってる!" << endl;
  }

  return 0;
}
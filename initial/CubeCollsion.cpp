#include <iostream>
#include <cmath>
using namespace std;

typedef struct{
  double x;
  double y;
  double z;
} POINT;


void Link(const double xMin, const double xMax,
          const double yMin, const double yMax,
          const double zMin, const double zMax, POINT *P){

  P[0].x = xMin; P[0].y = yMin; P[0].z = zMin;
  P[1].x = xMax; P[1].y = yMin; P[1].z = zMin;
  P[2].x = xMax; P[2].y = yMax; P[2].z = zMin;
  P[3].x = xMin; P[3].y = yMax; P[3].z = zMin;
  P[4].x = xMin; P[4].y = yMin; P[4].z = zMax;
  P[5].x = xMax; P[5].y = yMin; P[5].z = zMax;
  P[6].x = xMax; P[6].y = yMax; P[6].z = zMax;
  P[7].x = xMin; P[7].y = yMax; P[7].z = zMax;

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
  } else {
    compare[0].x = A.x;
    compare[1].x = B.x;
  }

  if ((A.y - B.y) > 0) {
    compare[0].y = B.y;
    compare[1].y = A.y;
  } else {
    compare[0].y = A.y;
    compare[1].y = B.y;
  }

  if ((A.z - B.z) > 0) {
    compare[0].z = B.z;
    compare[1].z = A.z;
  } else {
    compare[0].z = A.z;
    compare[1].z = B.z;
  }
}


bool Intersection(const double xMin, const double xMax,
                  const double yMin, const double yMax,
                  const double zMin, const double zMax,
                  POINT A, POINT B){

  int i = 0;
  bool flag;
  POINT P, p[8], E, compare[2];
  double a[4];
  double t, DE;
  int i0[6] = {0,0,1,2,3,4};
  int i1[6] = {1,1,2,3,4,5};
  int i2[6] = {2,4,5,6,7,6};

  Link(xMin, xMax, yMin, yMax, zMin, zMax, p);

  do{
    PlaneEquation(p, i0, i1, i2, i, a);
    E.x = A.x - B.x;
    E.y = A.y - B.y;
    E.z = A.z - B.z;
    Pcompare(A, B, compare);


    t = (a[3] - (a[0] * A.x + a[1] * A.y + a[2] * A.z)) /
        (a[0] * E.x + a[1] * E.y + a[2] * E.z);
    DE = a[0] * E.x + a[1] * E.y + a[2] * E.z;

    if(DE == 0){
      cout << i << "面と直線との" << "交点なし" << endl;
      flag = true;
    }else{
      //cout << "交点あり" << endl;
      P.x = A.x + t * E.x;
      P.y = A.y + t * E.y;
      P.z = A.z + t * E.z;
      cout << i << ", (" << P.x << ", " << P.y << ", " << P.z << ")" << endl;

      if (xMin <= P.x && P.x <= xMax && yMin <= P.y && P.y <= yMax && zMin <= P.z && P.z <= zMax &&
          compare[0].x <= P.x && P.x <= compare[1].x &&
          compare[0].y <= P.y && P.y <= compare[1].y &&
          compare[0].z <= P.z && P.z <= compare[1].z ) {
        cout << "定義した4点内に入っている" << endl;
        flag = false;
        break;
      } else {
        //cout << "範囲外" << endl;
        flag = true;
      }
    }
    i++;
  }while(i<6);

  return flag;
}//


int main(){
  POINT start = {0,2,1.5};
  POINT end = {4,2,1.5};;
  bool output;

  output = Intersection(2, 3, 2, 3, 2, 3, start, end);
  if(output == true){
    cout << "当たってない!" << endl;
  }else{
    cout << "当たってる!" << endl;
  }

  return 0;
}
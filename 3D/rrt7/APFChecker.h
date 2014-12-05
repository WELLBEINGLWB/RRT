#ifndef FUNCTION_H_
#define FUNCTION_H_

#include "RRTStruct.h"
#define MAX_FIELD 4

/*-----------------ポテンシャルの係数-----------------*/
#define K 60        // 障害物のポテンシャルの高さ
#define K_1 6       // goalに導く引力ポテンシャルの高さ
#define r_1 3     // ポテンシャルのx軸方向の大きさ
#define r_2 3     // ポテンシャルのy軸方向の大きさ
#define r_3 3     // ポテンシャルのz軸方向の大きさ


class APF {
  public:
    APF(POINT s, POINT g, string fileName);
    APF(POINT s, POINT g, string fileName, RANGE o);

    void InitialCondition();
    int CountNumbersOfTextLines(string fileName);
    void Input_Data(string fileName);
    void CreateCube(RANGE obstacle, string fileName);

    double f_xyz(double x, double y, double z);
    double fx(double x, double y, double z);
    double fy(double x, double y, double z);
    double fz(double x, double y, double z);
    //void gridf(double xs[], double ys[], double zs[]);

    POINT extreme(double xs[], double ys[], double zs[], string dat_output);
    void output_result(POINT p);
    void output_plt(string plt_output);
    void output_plt(string plt_output, string cube_output);

  private:
    vector< POINT > vobstacle;
    POINT start;
    POINT goal;
    RANGE obstacle;
};
#endif
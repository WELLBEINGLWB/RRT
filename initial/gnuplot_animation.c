#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

void gnuplot_config(FILE *gp);
void plot2d(FILE *gp,const int num,const double *x,const double *y);
void move(const int num, double const t, double *x, double *y);

int main(void){
    FILE *gp;

    //gp = popen("gnuplot -persist", "w");    /*プログラムが終了後GnuplotのWindowを閉じない*/
    gp = popen("gnuplot", "w");               /*プログラムが終了後GnuplotのWindowを閉じる*/
    gnuplot_config(gp);  /* gnuplot configuration */


    int i;          // loop variable
    int step=100;
    int sample_num=10;
    double t;
    double dt=0.01;
    double x[sample_num], y[sample_num];

    printf("途中で終了する場合は，Ctrl-C をターミナルに入力\n");

    for (i=0;i<step;i++){
        fprintf(gp,"set title 'animated-%d/%d' font 'Times, 20'\n",i,step);
        t = i*dt;
        move(sample_num, t, x, y);
        plot2d(gp,sample_num, x, y);
        usleep(30000); //指定された(マイクロ?)秒プログラムを停止させます (Unix system only)
//        while(getchar() != '\n');       /*Enter keyが入力されるまで，計算停止*/
    }
    printf("計算終了\n");


    return 0;
}

void gnuplot_config(FILE *gp)
{

    fprintf(gp,"set terminal gif animate optimize size \n");
    fprintf(gp,"set output 'anime.gif'\n");

    /*アニメーションをgif動画として保存する場合は，上のコメントを解除*/
    fprintf(gp,"set xrange [-1.5:1.5]\n");
    fprintf(gp,"set yrange [-1.5:1.5]\n");
    fprintf(gp,"unset tics\n");
//    fprintf(gp,"set xtics 0.5\n");
//    fprintf(gp,"set ytics 0.5\n");
//    fprintf(gp,"set xlabel 'x' font 'Times, 20'\n");
//    fprintf(gp,"set ylabel 'y' font 'Times, 20'\n");
//    fprintf(gp,"set grid\n");
    fprintf(gp,"set border 0\n");
    fprintf(gp,"set size square\n");
}

void plot2d(FILE *gp, const int num, const double *x, const double *y)
{
    int i;
    fprintf(gp,"plot");
    for(i=0;i<num;i++){
        if (i!=num-1)
            fprintf(gp,"'-' with p pt 7 ps 10 lc %d tit '',", i);//plotデータの読み込み開始
        else
            fprintf(gp,"'-' with p pt 7 ps 10 lc %d tit ''\n", i);//plotデータの読み込み開始
    }
    for(i=0;i<num;i++){
        fprintf(gp,"%f %f\n",x[i],y[i]);
        fprintf(gp,"e\n"); //データ書き込み終了を知らせる．
    }
    fflush(gp); //バッファーに溜まったデーターを吐き出す．これをしないと，real time animation にならない．
}


void move(const int num, const double t, double *x, double *y)
{
    int i;
    double init_phase[num];

    for (i =0;i<num;i++){
        init_phase[i] =  i*2.0*M_PI/num;
    }

    for (i=0;i<num;i++){
        x[i] = cos(t+init_phase[i]);
        y[i] = sin(t+init_phase[i]);
    }
}
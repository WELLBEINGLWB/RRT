# RRT

* RRT(Rapidly exploring Random Tree)のコード
* 2次元版と3次元版がある
* 最近では人工ポテンシャル場と組み合わせたやつもある
* branchは「T-RRT」

## [1. 2D](https://github.com/Ry0/RRT/tree/APF_Collision_Check/2D)
RRTの2Dバージョン  
RRTをやり始めた最初のプログラムもあるし、RRTの原理を説明するための幹を伸ばす一連を1ループごとにプロットするプログラムもあり  
rrt5が一番まともかな  
![RRT2D](https://dl.dropboxusercontent.com/u/23873125/Markdown/rrt2d.jpg)

### rrt
* RRTの一番最初のソースプログラム
* 元データは[ここから(The University of North Carolina at Chapel Hill)](http://www.cs.unc.edu/~luis/courses/robotics/)
* おそらく何もいじっていない

### rrt2
* 実際に木の幹のプロットデータを出力するようにしたもの

### rrt3
* 本格的にRRTをやり始めたころのやつ
* 元のコメントアウトに日本語を追加したりしてる
* GNU Plotに直接パイプを通してやろうとしているがおそらく動かない
* まあ使えない

### rrt4
* GNU Plot側で定期的にファイル更新を見るようにして、プロットが動いているようにしている
* プログラムも以前より見やすくなっている
* `randFreeSample(double* x, double* y)`の`*x`と`*y`を一定確率でゴール地点にすることで圧倒的に局所解に陥らなくなった

```cpp
void MotionPlan::RRT::randFreeSample(double* x, double* y)
{
  int roulette;

  //srand((unsigned int)time(NULL));
  roulette = (((double)rand())/RAND_MAX)*100;
  //std::cout << "ルーレットは" << roulette << "です！" << std::endl;
  if(roulette >= 90){
    (*x) = xGoal;
    (*y) = yGoal;
  }else{
    do{
      (*x) = (((double)rand())/RAND_MAX)*(xRight - xLeft) + xLeft;
      (*y) = (((double)rand())/RAND_MAX)*(yTop - yBottom) + yBottom;
    } while(!clear(xMin, xMax, yMin, yMax, numObstacles, *x, *y));
  }

}
```
* 経路の洗練を行うアルゴリズムの追加（HCフィルタ）
```cpp
  void RRTloop(int* iterations, int* nodePath, int* pathLength, std::ostream& nodeData);
  int GetRandom(double min, double max);
  double Distance();
  void smoothing(int loop);
```
* 最終的に`RRTloop(int* iterations, int* nodePath, int* pathLength, std::ostream& nodeData)`に全部の関数を入れるようにして、main文をスマートに
* 障害物が動いても対応できるようなものも多少作っている。ロボットアームがどこまで動いているかっていうのが全く定義しきれていないから、実際には使えない。

### rrt5
* branch、「[APF_Collision_Check](https://github.com/Ry0/RRT/tree/APF_Collision_Check/2D/rrt5)」にて衝突判定において人工ポテンシャル法を使っている。
* ただし現段階(2014.12.15)では実行速度が幾何学的に衝突判定を行っている従来の方法と比べてかなり遅いのでそこが問題  

```cpp
  bool clear(double xTest, double yTest, std::vector<POINT> &vobstacle);

  bool link(double xStart, double yStart,
            double xDest, double yDest,
            std::vector<POINT> &vobstacle, double stepSize);

  double f_xy(double x,double y, std::vector<POINT> &vobstacle);
```

* rrt4とくらべてスプライン関数を用いた平滑処理も加えたことから、ファイルの分割も多くなっている
* スプライン曲線を引いたあとの衝突判定に関しても、ポテンシャル法によってある程度ぶつかっているかどうかの判定は可能（実行速度はどうだろ...）`class Draw { }`で実装
* あとオマケ程度にプログラムの実行時の引数を指定してエラー処理をする関数を別で書いた
* (2015.1.16)T-RRTの実装を開始。
* 参考文献：![http://www.iri.upc.edu/people/ljaillet/Papers/Iros08_TransitRRT_final.pdf](http://www.iri.upc.edu/people/ljaillet/Papers/Iros08_TransitRRT_final.pdf)  

### rrt_check
* 単純にrrtがどのようにゴールまで幹を伸ばしていくかっていうのを1ループごとに見るためのプログラム
* 要OpenCV

### rrt_check2
* 障害物が動いたときどうなるのか障害物の動く1周期ごとにcvWaitKeyで待って観察するもの
* なんか残念


## [2. 3D](https://github.com/Ry0/RRT/tree/APF_Collision_Check/3D)
* 2Dでやったことを3次元に拡張したやつ
* 2Dでやってきたノウハウを活かしたものが多い
* 3次元での衝突判定は2次元のときと都合が違ってくるので、そこは大きく変更している
* 3次元でもAPFを組み込んでみる予定
* 2014.12.15 時点でrrt7が一番新しいやつ

![rrt3d](https://dl.dropboxusercontent.com/u/23873125/Markdown/rrt3d_mod.jpg)

### rrt4
* はじめて3次元においてRRTを実装した記念すべき初号機
* 下のソース部分で3次元でも衝突判定ができている
* 基本は3次元空間での直線と平面の交点の導出からぶつかっているかどうかを判定している

```cpp
  bool clear(const double* xMin, const double* xMax,
             const double* yMin, const double* yMax,
             const double* zMin, const double* zMax,
             int numObstacles,
             double xTest, double yTest, double zTest);

  void CreateGridPoint(const double* xMin, const double* xMax,
                       const double* yMin, const double* yMax,
                       const double* zMin, const double* zMax, POINT *P, int i);

  void PlaneEquation(POINT p[], int i0[], int i1[], int i2[], int i, double a[]);

  void Pcompare(POINT A, POINT B, POINT *compare);

  bool link(const double* xMin, const double* xMax,
            const double* yMin, const double* yMax,
            const double* zMin, const double* zMax,
            int numObstacles,
            double xStart, double yStart, double zStart,
            double xDest, double yDest, double zDest);

  void CreateGrid(const double xMin, const double xMax,
                  const double yMin, const double yMax,
                  const double zMin, const double zMax, POINT *P);

  void PlaneEquation(POINT p[], int i0[], int i1[], int i2[], int i, double a[]);

  void Pcompare(double xStart, double yStart, double zStart,
                double xDest, double yDest, double zDest, POINT *compare);

  bool Intersection(const double xMin, const double xMax,
                    const double yMin, const double yMax,
                    const double zMin, const double zMax,
                    double xStart, double yStart, double zStart,
                    double xDest, double yDest, double zDest);

```
* まだこの時にはランダムサンプリングの改良ができていなかったので、局所解に陥りがち
* ループ回数も多め  
![rrt3d](https://dl.dropboxusercontent.com/u/23873125/Markdown/rrt3d.jpg)

### rrt5
* かなりマイナーアップデート
* rrt4でのは実装できていなかったランダムサンプリングのくだりを盛り込んだだけじゃないかな  
![rrt3dmod](https://dl.dropboxusercontent.com/u/23873125/Markdown/rrt3d_mod2.jpg)
```cpp
  roulette = (((double)rand())/RAND_MAX)*100;
  if(roulette >= 90){
    (*x) = xGoal;
    (*y) = yGoal;
    (*z) = zGoal;
  }else{
    ......
  }
```


### rrt6
* `bool MotionPlan::link( )`関数のバグ修正
* 2Dバージョンでもやってたようなことを3次元でもやっている
* 経路の洗練とB-Splineによる平滑化処理
* ソースファイルの分割、あとプログラム実行時の引数の指定と実行時間の計測

### rrt7
* rrt6で実装した平滑化処理のあと一応ポテンシャル法によってスプライン曲線が障害物とぶつかっていないかどうかを見るようにした
* `class Draw{ }`クラスの改善
* `#define PlotAnimation`による切り替え
* ~~2014.12.16 APFによる衝突判定をささっと導入した~~
* ~~簡単な障害物に関しては簡単に成功している（割と速い）~~
* 2014.12.18 全然見当違い。動いてねえよ！！  
* 結局修正できてまあ動いてる

## [3. initial](https://github.com/Ry0/RRT/tree/APF_Collision_Check/initial)
実際に適応するまえのテスト段階のソースコードたち  
3次元の衝突判定だったり、経路の洗練の部分だったり、スプライン処理だったり、APFの衝突判定だったり...

















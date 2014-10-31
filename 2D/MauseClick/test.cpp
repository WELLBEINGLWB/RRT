#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

// グローバル変数
bool click_flag = false;
// コールバック関数
void my_mouse_callback(int event, int x, int y, int flags, void* param)
{
  switch (event) {
    case cv::EVENT_LBUTTONDOWN:
      click_flag = true;
      break;

    // case cv::EVENT_LBUTTONUP:
    //   click_flag = false;
    //   break;
  }
}

int main(void)
{
  int i = 0;
  char filename[] = "no_shintyoku.png";  //ここに相対パスでも絶対パスでもいいので指定したい画像ファイルを書いてください";
                           ////いじるのはここだけです。

  IplImage* img = cvLoadImage(filename, CV_LOAD_IMAGE_ANYCOLOR);
  if (img == NULL) {
    fprintf(stderr, "なんの成果も得られませんでしたぁあああああああ！\n");
    exit(1);
  }

  // ウィンドウを生成
  cvNamedWindow("進撃のテスト", CV_WINDOW_AUTOSIZE);
  cvShowImage("進撃のテスト", img);
  // コールバックを設定
  cv::setMouseCallback("進撃のテスト", my_mouse_callback, img);

  while (1) {
    // Main loop
    while (1) {
      // マウスの左クリックを離すまでの間、矩形を一時的に描画
      if (click_flag) {
        cout << i << "ループ目、左クリック押してる！" << endl;
        click_flag = false;
        break;
      }
      // Escで終了
      if (cv::waitKey(15) == 27){
        break;
      }
    }

    i++;

  }

  cvReleaseImage(&img);
  cvDestroyWindow("進撃のテスト");

  return 0;
}
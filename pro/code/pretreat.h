#ifndef PRETREAT_H_
#define PRETREAT_H_
#include "opencv_hpp.h"
void Cut_img(Mat src_img, int m, int n, Mat *imgL, Mat *imgR);
void Normalization(Mat srcImage, Mat *putImage);
Mat equalizeIntensityHist(const Mat &inputImage);
void getRG(Mat src_img, Mat *RG); //过滤绿色
void Enhancement(Mat src_img, Mat *enhancement);
void Segment(Mat src_img, Mat *seg);
int OTSU(Mat src);
void adjust_img(Mat src_img, Mat *adjust_img);
Mat filledROIMat(Mat srcImage0, Rect rectROI);
void tianchong_img(Mat src_img, Mat *adjust_img);
void test_cut(Mat img);
void test_tianchong(Mat img);
Mat Filling(Mat srcImg, Rect rectROI1, Rect rectROI2);
#endif

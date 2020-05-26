#ifndef CUT_IMG_H_
#define CUT_IMG_H_
#include <iostream>  
#include <vector>  
#include<string>
#include<sstream>
#include <algorithm>
#include "opencv_hpp.h"
using namespace std;
using namespace cv;

class Pig_measurement //猪体尺信息
{
public:
  float m_fButtock = 0;
  float m_fLength = 0;
  float m_fWidth = 0;
  float m_fHeigth = 0;
};
void Cut_img(Mat src_img, int m, int n, Mat* imgL, Mat* imgR);
void Xianweilan(Mat src_img, Rect rect, Rect rectL, Rect rectR, Mat* tianchong);
void Image_cut(Mat src,Mat *out);
void adjust_img(Mat src_img, Mat *adjust_img); // you bug
void detector_img(Mat src,Mat *img);
Pig_measurement Get_data(Mat src, Mat *img);
bool cmp_Y(Point A, Point B);
bool cmp_X(Point A, Point B);
float Get_Buttocks_width(vector<Point> Buttocks);
float Get_Length(vector<Point> point);
float Get_Width(vector<Point> point);
float Get_Heigth(vector<Point> point);
void find_ear_piont(vector<Point> p);

#endif
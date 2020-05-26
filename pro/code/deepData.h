#ifndef DEEPDATA_H_
#define DEEPDATA_H_
#include <opencv2/opencv.hpp>  
#include <iostream>
#include <iostream>  
#include <vector>  
#include<string>
using namespace std;
using namespace cv;
#define  ERROR_LEN  160000
void GenerateFalseMap(cv::Mat &src, cv::Mat &disp); //颜色变换
void stereo_match_bm(int, void *);
void stereo_match_sgbm(int, void *);
cv::Vec<float, 3> Deep_data(cv::Point Body_high_measur_point, Mat img_L, Mat img_R);
vector<Point> tall_point(Point T_point);
float Deal_Deep_Data(vector<Point> p, Mat img_L, Mat img_R); //这里形参必须是点集，然后进行点集求解，然后进行处理运算。
bool cmp_data(float x, float y);
static void onMouse(int event, int x, int y, int, void *);
#endif
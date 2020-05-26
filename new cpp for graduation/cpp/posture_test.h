#ifndef POSTURE_H_
#define POSTURE_H_
#include <iostream>  
#include <vector>  
#include<string>
#include "opencv_hpp.h"
using namespace std;
using namespace cv;
int otsu_threshold(Mat gray);

void Segment(Mat src_img, Mat* seg);

#endif
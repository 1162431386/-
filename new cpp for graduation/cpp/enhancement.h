#ifndef ENHANCEMENT_H_
#define ENHANCEMENT_H_
#include <iostream>  
#include <vector>  
#include<string>
#include "opencv_hpp.h"
using namespace std;
using namespace cv;
void Enhancement(Mat src_img, Mat* enhancement);

void getRG(Mat src_img, Mat* RG);

Mat equalizeIntensityHist(const Mat & inputImage);


#endif
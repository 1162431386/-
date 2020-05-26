#ifndef SIZE_CALCULAT_H_
#define SIZE_CALCULAT_H_
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <map>
#include "opencv_hpp.h"
#define LENGTH_FACTOR   1  //转化系数
typedef struct 
{
    float dist;
    int x;
    int y;
} TEST_LOCAL;   //尋找耳根點結構體

typedef struct 
{
    float dist;
    int x;
    int y;
} TAIL_LOCAL;

typedef struct 
{
    float Body_Buttocks;
    float Body_Length;
    float Body_Weigth;
    float Body_Tall;
}BODY_DATA;

typedef struct   /*實際尺寸*/
{
    float Body_Buttocks_T;
    float Body_Length_T;
    float Body_Weigth_T;
    float Body_Tall_T;
      
}TRUE_SIZE;


void Make_imgConvex(Mat src, Mat *out);
bool cmp(Point A, Point B);
float Get_Buttocks_width(vector<Point> Buttocks, Mat img);
float Get_Body_weigth(vector<Point> Buttocks, Mat img);
bool judge_posture(vector<Point> _right, vector<Point> _left, int mid_value);
bool cmp_x(Point A, Point B); //排序时的函数的参数以ｘ排序
float getDist_P2L(CvPoint pointP, CvPoint pointA, CvPoint pointB);
float getDistance(CvPoint pointO, CvPoint pointA);
void Convex(Mat src, Mat *out);
bool comp_vect(const TEST_LOCAL &st1, const TEST_LOCAL &st2);
bool comp_vect_tail(const TAIL_LOCAL &t1, const TAIL_LOCAL &t2);
bool HeadLow_judge(vector<Point> posturepoint);
void Coordinate_con(TRUE_SIZE *TrueSize, BODY_DATA BodyLocal);
#endif
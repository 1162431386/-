/*****************(2020 HZAU graduation project)*************
 * @file main.c
 * @author  wangwenkai <qq:1162431386>
 * @brief 主函数
 * @date 2020-05-22
 * @copyright Copyright (c) 2020
 */

/************************声明*********************************                    
 * 1.该套程序在linux下开发编译，基于g++编译器。若要进行跨平台
 * 2.移植，请注意c++版本问题，以及编译器问题。
 * 3.运行该程序需要安装opencv3.2.0以上版本的库。
 * 4.需要支持对于(c99)的编译
 * 5.Makefile文件已经实现将该程序打包静态库，可以实现跨平台移植
 * 6.对于算法优化可根据需要在此基础上更改
 * 7.注释均为UTF-8格式，若遇到注释乱码，可自行转化
 * **********************************************************/
#include "opencv_hpp.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include "pretreat.h"
#include "size_calculat.h"
#include "deepData.h"

using namespace std;
using namespace cv;
extern Point body_tall;     //体高测点     为保证准确性，去轮廓中心矩周围九个点
extern BODY_DATA Body_Data; //保存检测数据
TRUE_SIZE *True_Body_size = new TRUE_SIZE;
Mat imgL, imgR; //原始图像裁剪后图像
Mat enhancement_R;
Mat enhancement;
Mat Cut_img_L; //裁剪后的左图
Mat Cut_img_R;
Mat segment_R; //分割后图像
Mat filling;
Mat result; //边缘检测图
extern Mat Q;

int main()
{

   Mat start_img;
   ofstream outfile;                                //创建一个文件操作类
   outfile.open("../result/result.txt");            //打开此目录下的txt文档，没有的话，则自动创建一个，不需要用户手创建
   string dir_path = "../img/*.png"; //图像存放路经。
   vector<String> fileNames;
   outfile << "###############################实验结果显示说明######################################" << endl; //写入数据到txt文本*/
   outfile << "##                                                                                 ##" << endl; //写入数据到txt文本*/
   outfile << "## 正常情况下均以大于0的自然数显示，若显示-1则说明本次测试为不可用图像或者测试失败 ##" << endl; //写入数据到txt文本*/
   outfile << "##                                                                                 ##" << endl; //写入数据到txt文本*/
   outfile << "#####################################################################################" << endl; //写入数据到txt文本*/
   glob(dir_path, fileNames);
   for (int i = 0; i < fileNames.size(); i++)
   {
      start_img = imread(fileNames[i]);
      Cut_img(start_img, 1, 2, &imgL, &imgR);             //裁剪为左右两图
      Cut_img_L = imgL(Range(139, 614), Range(543, 672)); //裁剪左
      Cut_img_R = imgR(Range(130, 631), Range(453, 636)); //裁剪右
      //cvtColor(imgR, imgR, COLOR_BGR2GRAY);
#if 0
      /*******************************
     * 注意：该段用于动态寻找目标，裁剪图像
     * 找到相应阈值后可直接放到Rang函数里。
     * *****************************/
       test_cut(imgR);
    
      /*该函数用于得到填充限位栏的rect的坐标*/
       test_tianchong(Cut_img_R);
#endif
      Enhancement(start_img, &enhancement);                          /*直方图均衡化*/
      enhancement_R = enhancement(Range(130, 631), Range(453, 636)); //裁剪右
      medianBlur(enhancement_R, enhancement_R, 3);
      cvtColor(enhancement_R, enhancement_R, COLOR_BGR2GRAY);
      filling = Filling(enhancement_R, Rect(0, 0, 30, 536), Rect(157, 0, 204, 536));     //限位栏填充
      Segment(filling, &segment_R);                                                      //OTSU阈值
      Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));                    //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
      morphologyEx(segment_R, segment_R, MORPH_OPEN, element);                           /*高级形态学处理，调用这个函数就可以了，具体要选择哪种操作，就修改第三个参数就可以了。这里演示的是形态学梯度处理*/
                                                                                         /********************************************************
      * 图像预处理完毕，进行轮廓绘制和测点匹配
      * *******************************************************/
      Make_imgConvex(segment_R, &result);                                                //形态学处理和边缘检测寻找轮廓 面积筛选 最小凸包
      Body_Data.Body_Tall = Deal_Deep_Data(tall_point(body_tall), Cut_img_L, Cut_img_L); //这里形参必须是点集，然后进行点集求解，然后进行处理运算。
      Coordinate_con(True_Body_size, Body_Data);                                         //用于坐标转化，将像素转化为实际坐标，具体更改Size_calculat.h的长度系数宏

      //imshow("Enhancement_R", enhancement_R);  //直方图均衡化
      //imshow("segment", segment_R); 
      // imshow("cut_imgR", filling);              //限位栏填充图
      // imshow("result", result);
      cout << True_Body_size->Body_Length_T << endl;
      cout << "***********第" << i << "张读取完毕**********" << endl;
      cout << "正在写入文件......" << endl;
      outfile << fileNames[i] << ";"
              << "体长;" << True_Body_size->Body_Length_T << ";"
              << "体宽;" << True_Body_size->Body_Weigth_T << ";"
              << "臀宽;" << True_Body_size->Body_Buttocks_T << ";"
              << "体高;" << True_Body_size->Body_Tall_T << ";" << endl; //写入数据到txt文本*/
     // waitKey(1000);
     
   }
   outfile.close(); //关闭txt文本
   delete True_Body_size;
   return 0;
}

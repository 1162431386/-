#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "order.h"
#include "uart.hpp"
using namespace std;
using namespace cv;
Mat img_brfore;
Mat img_next;
Mat img_result;
Mat img;
int main()
{
   create_thread(); //创建线程
   VideoCapture capture;
   capture.open(0);
   if (!capture.isOpened())
   {
      cout << "No capture" << endl;
      return -1;
   }
   else
   {
      for (;;)
      {
         
         capture >> img;
         imshow("img", img);
         if (waitKey(1) == 27)
            break;
      }
   }
   
  return 0;
}

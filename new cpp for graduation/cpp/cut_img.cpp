#include "cut_img.h"
/*感性区域调节参数 */
void Cut_img(Mat src_img, int m, int n, Mat* imgL, Mat* imgR)
{
	int t = m * n;
	int height = src_img.rows;
	int width = src_img.cols;

	int ceil_height = height / m;
	int ceil_width = width / n;
	Mat roi_img;

	for (int i = 0; i<m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			Rect rect(j*ceil_width, i*ceil_height, ceil_width, ceil_height);
			if (j == 0)
			{
				src_img(rect).copyTo(*imgR);
			}
			else
			{
				src_img(rect).copyTo(*imgL);
			}
		}
	}
}



/* void Rectify(Mat imgL, Mat imgR, Mat* rectifyimgL, Mat* rectifyimgR)
{
	//首先这里利用matlab自带的双目标定工具对摄像头进行标定，操作简单。获得标定数据 具体参考
	 // https://blog.csdn.net/hanshihao1336295654/article/details/82719511
	 Rodrigues(rec, R); //Rodrigues变换 这一步必须执行
	 stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
		0, imageSize, &validROIL, &validROIR);

	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
	
	
	remap(imgL, *rectifyimgL, mapLx, mapLy, INTER_LINEAR);
	remap(imgR, *rectifyimgR, mapRx, mapRy, INTER_LINEAR);
	
	imshow("rectifyImageL", *rectifyimgL);
	imshow("rectifyImageR", *rectifyimgR);
}
*/

/* void adjust_img(Mat src_img, Mat *adjust_img) // you bug

{   
     namedWindow("dst", WINDOW_GUI_EXPANDED);

	 createTrackbar("left_l", "dst", &left_l, left_l_max);

	 createTrackbar("right_l", "dst", &right_l, right_l_max);

	 createTrackbar("up_l", "dst", &up_l, up_l_max);

	 createTrackbar("down_l", "dst", &down_l, down_l_max);
	
    *adjust_img = src_img(Range(up_l,down_l),Range(left_l,right_l));
}
*/

void Image_cut(Mat src,Mat *out)
{
      
     // RGB_to_HSV(just_img,&HSV_img);
     //   Mat out;
     //获取自定义核
     Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
        
      //高级形态学处理，调用这个函数就可以了，具体要选择哪种操作，就修改第三个参数就可以了。这里演示的是形态学梯度处理
      morphologyEx(src, src, MORPH_OPEN, element);
      morphologyEx(src, src, MORPH_OPEN, element);
      morphologyEx(src, src, MORPH_OPEN, element);
      morphologyEx(src, src, MORPH_CLOSE, element); 
      //cvtColor(src, src, COLOR_BGR2GRAY);
      //cvtColor(enhancement, HSV_img, COLOR_BGR2GRAY);
	    threshold(src, src, 70, 255,CV_THRESH_BINARY);
      vector<vector<Point>> contours;
      vector<Vec4i> hierarchy;

      findContours(src, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0)); //找轮廓

      for (int i = 0; i < contours.size(); i++)

      {
          double ConArea = contourArea(contours[i], true);
          cout << "【用轮廓面积计算函数计算出来的第" << i << "个轮廓的面积为：" << ConArea << endl; ///计算面积
    } 

    /*****************************************
     * 用于面积的筛选，可以在一定取值内剔除不相关的值.
     * ************************************/   
     
     vector <vector<Point>>::iterator iter = contours.begin();
     for (; iter != contours.end();)
     {
         double g_dConArea = contourArea(*iter);
     if (g_dConArea < 300)
     {
         iter = contours.erase(iter);
         
     }
     else
     {
         ++iter;
         }}
     cout << "【筛选后总共轮廓个数为：" << (int)contours.size() << endl;
 
      vector<Moments> mu(contours.size() );       

    for( int i = 0; i < contours.size(); i++)     

    {   

        mu[i] = moments( contours[i], false );   

    }     

    //ŒÆËãÂÖÀªµÄÖÊÐÄ     

    vector<Point2f> mc( contours.size() );      

    for( int i = 0; i < contours.size(); i++ )     

    {   

        mc[i] = Point2d( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );   

    }     

    //计算中心距     

       Mat drawing = Mat::zeros( src.size(), CV_8UC3 );   
       Point2f center;  //定义圆中心坐标
       float radius;  //定义圆半径      

    for( int i = 0; i< contours.size(); i++ )      

    {         

        Scalar color = Scalar( 255, 0,0);  //颜色通道赋值     
        minEnclosingCircle(Mat(contours[i]), center, radius);
        drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );   //绘制轮廓        
        rectangle(drawing, boundingRect(contours.at(i)), cvScalar(0,255,0));  //
		Rect rRect =  boundingRect(contours.at(i));
        cout<<"向量hierarchy的第" <<i<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;
		cout<<rRect<<endl;    
    }
      //采集数据处理
   *out=drawing;
}

Pig_measurement Get_data(Mat src, Mat *img)
{
   static Pig_measurement m_Data;
    // RGB_to_HSV(just_img,&HSV_img);
    //   Mat out;
    //获取自定义核
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的

    //高级形态学处理，调用这个函数就可以了，具体要选择哪种操作，就修改第三个参数就可以了。这里演示的是形态学梯度处理
    morphologyEx(src, src, MORPH_OPEN, element);
    morphologyEx(src, src, MORPH_OPEN, element);
    morphologyEx(src, src, MORPH_OPEN, element);
    morphologyEx(src, src, MORPH_CLOSE, element);
    //cvtColor(src, src, COLOR_BGR2GRAY);
    //cvtColor(enhancement, HSV_img, COLOR_BGR2GRAY);
    threshold(src, src, 70, 255, CV_THRESH_BINARY);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(src, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0)); //找轮廓

    double Max_Area = 0; //保存当前轮廓最大面积
    for (int i = 0; i < contours.size(); i++)

    {
        double ConArea = contourArea(contours[i], true);
        if (fabs(Max_Area) < fabs(ConArea))
        {
            Max_Area = ConArea;
        }
        //cout << "Max_"<<Max_Area<< endl; ///计算面积
        //cout << "***" << ConArea << endl; ///计算面积
     } 

    /*****************************************
     * 用于面积的筛选，可以在一定取值内剔除不相关的值.
     * ************************************/   
     
     vector <vector<Point>>::iterator iter = contours.begin();
     for (; iter != contours.end();)
     {
         double g_dConArea = contourArea(*iter);
     if (g_dConArea < fabs(Max_Area))
     {
         iter = contours.erase(iter);
         
     }
     else
     {
         ++iter;
         }}
     //cout << "【筛选后总共轮廓个数为：" << (int)contours.size() << endl;
 
      vector<Moments> mu(contours.size() );       

    for( int i = 0; i < contours.size(); i++)     

    {   

        mu[i] = moments( contours[i], false );   

    }     

    //ŒÆËãÂÖÀªµÄÖÊÐÄ     

    vector<Point2f> mc( contours.size() );      

    for( int i = 0; i < contours.size(); i++ )     

    {   

        mc[i] = Point2d( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );  

    }      
    //   vector<vector<Point> >hull( contours.size() );   //找凸包
    //   vector<vector<int>> hullsI(contours.size());
    //   vector<vector<Vec4i>> hullDefect(contours.size()); //凸缺陷
    //   for (int i = 0; i < contours.size(); i++)

    //   {
    //       //convexHull(Mat(contours[i]), hull[i], false);
    //       //convexHull(Mat(contours[i]), hullsI[i], false);
    //       //    void cv::convexHull (   InputArray  points,
    //       //                            OutputArray     hull,
    //       //                            bool    clockwise = false,
    //       //                            bool    returnPoints = true  )
    //       //          points:输入的二维点集，Mat类型数据即可
    //       //          hull:输出参数，用于输出函数调用后找到的凸包
    //       //          clockwise:操作方向，当标识符为真时，输出凸包为顺时针方向，否则为逆时针方向。
    //       //           returnPoints:操作标识符，默认值为true，此时返回各凸包的各个点，否则返回凸包
    //       //    各点的指数，当输出数组时std::vector时，此标识被忽略。
    //       //
    //       //convexityDefects(Mat(contours[i]), hullsI[i], hullDefect[i]);


    // }
    //    计算中心距 


       Mat drawing= Mat::zeros( src.size(), CV_8UC3 ); 

       Point2f center;  //定义圆中心坐标
       float radius;  //定义圆半径
       int rectangle_y;  //外接矩形的长
       int rectangle_x;   //宽
       for (int i = 0; i < contours.size(); i++)
       {
           Scalar color = Scalar(255, 0, 0); //颜色通道赋值
           //minEnclosingCircle(Mat(contours[i]), center, radius);
           drawContours(drawing, contours, i, color, 1, 8, hierarchy, 0, Point()); //绘制轮廓
           // drawContours(drawing, hull, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point());
           rectangle(drawing, boundingRect(contours.at(i)), cvScalar(0, 255, 0)); //
           Rect rRect = boundingRect(contours.at(i));
           // cout << "向量hierarchy的第" << i << " 个元素内容为：" << endl
           //     << hierarchy[i] << endl
           //     << endl;
           rectangle_x = (rRect.br().x + rRect.tl().x);
           rectangle_y = fabs(rRect.br().y - rRect.tl().y);                                                                                                                //赋值宽度
           line(drawing, Point((rRect.br().x + rRect.tl().x) / 2, rRect.tl().y), Point((rRect.br().x + rRect.tl().x) / 2, rRect.br().y), cvScalar(0, 0, 255), 1, 1); //绘制中线
           //drawContours(drawing, hullDefect, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point()); //绘制轮廓
       }

       vector<Point> sort_point_x=contours[0];
       sort(sort_point_x.begin(), sort_point_x.end(), cmp_X); //以x为优先级排序，从小到大
        vector< Point> cross_point;     //轮廓中线和轮廓的交点
        vector<int> Error_x;            //所有轮廓点和中线的距离
        for (int i = 0; i < sort_point_x.size(); i++)
        {
            Error_x.push_back(rectangle_x / 2 - sort_point_x[i].x); //计算点到直线距离
            if (rectangle_x / 2 - sort_point_x[i].x == 0)

            {
                cross_point.push_back(sort_point_x[i]); //保存焦点到piont
           }          
           cout<< Error_x[i] << endl;
         // cout << cross_point<< endl;
         // cout << sort_point_x << endl;
        }

        
       vector<Point> Buttocks;           //臀侧轮廓
       vector<Point> Head;     //头侧
       vector<Point> sort_point_y = contours[0]; //轮廓坐标排序数组，以y坐标排序　,排序好就放在这里
       sort(sort_point_y.begin(), sort_point_y.end(), cmp_Y);  //以ｙ为优先级排序，从小到大
       for (int j = 0; j < sort_point_y[j].y;j++)
       {
           if (sort_point_y[j].y<rectangle_y)
           {
               Buttocks.push_back(sort_point_y[j]);
           }
           else
           {
               Head.push_back(sort_point_y[j]);
           }
        }
        m_Data.m_fButtock=Get_Buttocks_width(Buttocks); //臀宽
        *img = drawing;
        return m_Data;
}


bool cmp_Y(Point A, Point B)  //sort()回调函数　比较点大小
{
  if (A.y!=B.y)
  {
      return A.y < B.y;
  }
  else
  {
      return A.x < B.x;
  }
   
}

 bool cmp_X(Point A, Point B) //sort()回调函数　比较点大小
 {
     if (A.x != B.x)
     {
         return A.x < B.x;
     }
     else
     {
         return A.y < B.y;
     }
 }

     void find_ear_piont(vector<Point> p) //寻找耳根点

 {
   




 }

 float Get_Buttocks_width(vector<Point> Buttocks) //函数返回值为臀宽
 {
     float swap = 0;
     for (int i = 0; i < Buttocks.size(); i++)
     {
         int result = Buttocks[i].x - Buttocks[i + 1].x;
          if(result>swap)
              swap = result;
     }
     return swap;
 }

 float Get_Length(vector<Point> point)
 {
 


 }

 float Get_Width(vector<Point> point)
 {


 }

 float Get_Heigth(vector<Point> point)
 {


 }

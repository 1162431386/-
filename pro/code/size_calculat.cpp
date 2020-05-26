/**
 * @file size_calculat.cpp
 * @author  wangwenkai <qq:1162431386> 
 * @brief 测点匹配及长度计算
 * @date 2020-05-22
 * @copyright Copyright (c) 2020
 */

#include "size_calculat.h"
#include "deepData.h"
vector<Point> posture_point; //保存轮廓点信息
vector<TEST_LOCAL> local_array;
vector<TAIL_LOCAL> Tail_array;
Point body_tall;     //体高测点     为保证准确性，去轮廓中心矩周围九个点
BODY_DATA Body_Data; //保存检测数据  注意内存释放问题
void Make_imgConvex(Mat src, Mat *out)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(src, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0)); //找轮廓
    double Max_Area = 0;                                                                     //保存当前轮廓最大面积
    for (int i = 0; i < contours.size(); i++)

    {
        double ConArea = contourArea(contours[i], true);
        if (fabs(Max_Area) < fabs(ConArea))
        {
            Max_Area = ConArea;
        }
    }

    /*****************************************
     * 用于面积的筛选，可以在一定取值内剔除不相关的值.
     * ************************************/
    vector<vector<Point> >::iterator iter = contours.begin();
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
        }
    }

    vector<Moments> mu(contours.size());

    for (int i = 0; i < contours.size(); i++)

    {

        mu[i] = moments(contours[i], false);
    }

    vector<Point2f> mc(contours.size());

    for (int i = 0; i < contours.size(); i++)

    {

        mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }

    body_tall = Point(mc[0].x, mc[0].y); //体高测点

    //    计算中心距
    Mat drawing = Mat::zeros(src.size(), CV_8UC3);

    /**************************************************/
    // circle(drawing,body_tall, 8, cvScalar(0, 0, 255));   //绘制体高测点

    int rectangle_y = 0;
    int rectangle_x = 0;
    Point2f center; //定义圆中心坐标
    float radius;   //定义圆半径
    Rect rRect;
    for (int i = 0; i < contours.size(); i++)

    {
        Scalar color = Scalar(255, 0, 0); //颜色通道赋值
        //minEnclosingCircle(Mat(contours[i]), center, radius);
        drawContours(drawing, contours, i, color, 1, 8, hierarchy, 0, Point()); //绘制轮廓
                                                                                // drawContours(drawing, hull, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point());
        rectangle(drawing, boundingRect(contours.at(i)), cvScalar(0, 255, 0));  //
        rRect = boundingRect(contours.at(i));
        // cout << "向量hierarchy的第" << i << " 个元素内容为：" << endl
        //     << hierarchy[i] << endl
        //     << endl;
        // rectangle_x = (rRect.br().x + rRect.tl().x);
        rectangle_y = (rRect.br().y + rRect.tl().y) / 2;
        rectangle_x = (rRect.br().x + rRect.tl().x) / 2;                                                                                                          //赋值宽度
        line(drawing, Point((rRect.br().x + rRect.tl().x) / 2, rRect.tl().y), Point((rRect.br().x + rRect.tl().x) / 2, rRect.br().y), cvScalar(0, 0, 255), 1, 1); //绘制中线
                                                                                                                                                                  //drawContours(drawing, hullDefect, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point()); //绘制轮廓                                                                                                                                                             //cout << (rRect.br().x + rRect.tl().x) / 2 <<" "<< rRect.tl().y << endl;
    }

    /* vector< Point> cross_point;     //轮廓中线和轮廓的交代
        vector<int> Error_x;            //所有轮廓点和中线的距离
         for (int i = 0; i < contours[0].size(); i++)
        {
            Error_x.push_back(rectangle_x / 2 - contours[0][i].x);    //计算点到直线距离
            if (rectangle_x / 2 - contours[0][i].x == 0)

            {
                cross_point.push_back(contours[0][i]);    //保存焦点到piont
           }          
           cout<< Error_x[i] << endl;
           cout << cross_point.size() << endl;
           cout << contours[0] << endl;
        }*/
    vector<Point> Buttocks;                 //臀侧轮廓，以y坐标排序　,排序好就放在这里
    vector<Point> Head;                     //头侧
    vector<Point> sort_point = contours[0]; //轮廓坐标排序数组
    posture_point = contours[0];            //定义姿态判断运算的点集合
    /*特征点 */
    sort(sort_point.begin(), sort_point.end(), cmp); //以ｙ为优先级排序，从小到大
    for (int j = 0; j < sort_point.size(); j++)
    {
        if (sort_point[j].y > rectangle_y)
        {
            Buttocks.push_back(sort_point[j]);
        }
        else
        {
            Head.push_back(sort_point[j]);
        }
    }

    vector<Point> Body_Left;                                 //左侧轮廓
    vector<Point> Body_right;                                //右侧
    sort(posture_point.begin(), posture_point.end(), cmp_x); //以x为优先级排序，从小到大
    for (int j = 0; j < posture_point.size(); j++)
    {
        if (posture_point[j].x < rectangle_x)
        {
            Body_Left.push_back(posture_point[j]);
        }
        else
        {
            Body_right.push_back(posture_point[j]);
        }
    }

    /*******************************************
     * 按照比例划分测点区域，然后辅助测点匹配进行匹配，增加匹配
     * 准确度。
     * 注意：在进行比例划分前一定要判断姿态是否弯曲，是否低头
     * *****************************************/
    vector<Point> head_area;               //头部区域
    vector<Point> Auricular_area;          //耳根点区域
    vector<Point> Shoulder_width_area;     //肩宽区域
    vector<Point> Hip_width_area;          //臀宽域
    vector<Point> Tail_point_area;         //尾根点区域
                                           /********************************************
        * 耳根点范围0.1428~0.2197
        * 肩宽：0.2197~0.35164
        * 臀宽：0.696629~0.94382
        * 尾：0.94382~1
        * 均为占猪体总长的比例，算猪头
       *********************************************/
    int start_y = rRect.tl().y;            //记录y的起点坐标
    int len = rRect.br().y - rRect.tl().y; // 记录总长
    {
        int thead_H = start_y + len * 0.1428;    //耳根点
        int thead_A_l = start_y + len * 0.2197;  //肩宽起点 耳根终点
        int thead_A_h = start_y + len * 0.35164; //肩宽float
        int thead_S_l = start_y + len * 0.6966;  //臀宽起点
        int thead_S_h = start_y + len * 0.9438;  //臀宽起点

        for (int i = 0; i < sort_point.size(); i++)
        {
            if (sort_point[i].y < thead_H) //小于头部长度
            {
                head_area.push_back(sort_point[i]);
            }

            else if (sort_point[i].y >= thead_H && sort_point[i].y < thead_A_l)
            {
                Auricular_area.push_back(sort_point[i]);
            }

            else if (sort_point[i].y >= thead_A_l && sort_point[i].y < thead_A_h)
            {
                Shoulder_width_area.push_back(sort_point[i]);
            }

            else if (sort_point[i].y >= thead_S_l && sort_point[i].y < thead_S_h)
            {
                Hip_width_area.push_back(sort_point[i]);
            }
            else if (sort_point[i].y >= thead_S_h)
            {
                Tail_point_area.push_back(sort_point[i]);
            }
        }

        // cout << head_area.size() << endl;

#if 0
          /*绘制头部*/
          for (int i = 0; i < head_area.size();i++)
          {
               circle(drawing, Point(head_area[i].x, head_area[i].y), 2, cvScalar(0, 0, 255)); //绘制点
          }
          
          /*绘制耳根*/
          for (int i = 0; i < Auricular_area.size();i++)
          {
               circle(drawing, Point(Auricular_area[i].x, Auricular_area[i].y), 2, cvScalar(0, 0, 255)); //绘制点
          }

          /*绘制肩*/
           for (int i = 0; i < Shoulder_width_area.size();i++)
          {
               circle(drawing, Point(Shoulder_width_area[i].x, Shoulder_width_area[i].y), 2, cvScalar(0, 0, 255)); //绘制点
          }

          /*绘制臀*/
             for (int i = 0; i < Hip_width_area.size();i++)
          {
               circle(drawing, Point(Hip_width_area[i].x, Hip_width_area[i].y), 2, cvScalar(0, 0, 255)); //绘制点
          }

          /*绘制尾*/
            for (int i = 0; i < Tail_point_area.size();i++)
          {
               circle(drawing, Point(Tail_point_area[i].x, Tail_point_area[i].y), 2, cvScalar(0, 0, 255)); //绘制点
          }
#endif
    }

    /*找耳根体长测点辅助程序*/
    float dist_temp = 0;
    int Heda_size = Auricular_area.size();
    local_array.clear();
    for (int i = 0; i < Heda_size; i++)
    {
        TEST_LOCAL DIST_LOCAL;                                                                                                                                                                    //存放长度和坐标的结构体
        dist_temp = getDist_P2L(Point(Auricular_area[i].x, Auricular_area[i].y), Point((rRect.br().x + rRect.tl().x) / 2, rRect.tl().y), Point((rRect.br().x + rRect.tl().x) / 2, rRect.br().y)); //比较头部凸包与中线距离
        DIST_LOCAL.dist = dist_temp;
        DIST_LOCAL.x = Auricular_area[i].x;
        DIST_LOCAL.y = Auricular_area[i].y;
        local_array.push_back(DIST_LOCAL);
    }

    /*找尾根辅助函数*/
    float dist = 0;
    Tail_array.clear();
    for (int i = 0; i < Tail_point_area.size(); i++)
    {
        TAIL_LOCAL DIST_LOCAL;
        dist = getDist_P2L(Point(Tail_point_area[i].x, Tail_point_area[i].y), Point((rRect.br().x + rRect.tl().x) / 2, rRect.tl().y), Point((rRect.br().x + rRect.tl().x) / 2, rRect.br().y));
        DIST_LOCAL.dist = dist;
        DIST_LOCAL.x = Tail_point_area[i].x;
        DIST_LOCAL.y = Tail_point_area[i].y;
        Tail_array.push_back(DIST_LOCAL);
    }
    /*对结构体里面的长度进行排序，找耳根点*/
    sort(local_array.begin(), local_array.end(), comp_vect);    //按照从小到大排序，前两个为距离中心最小的轮廓点，
    sort(Tail_array.begin(), Tail_array.end(), comp_vect_tail); //按照从小到大排序，前两个为距离中心最小的轮廓点，
                                                                // circle(drawing, Point(local_array[0].x, local_array[0].y), 8, cvScalar(0, 0, 255)); //耳根点
                                                                // circle(drawing, Point(Tail_array[0].x,Tail_array[0].y), 8, cvScalar(0, 0, 255)); //尾根点
    int Bod_length = Tail_array[0].y - local_array[0].y;        //体长计算
                                                                /******************************************
       * 体长计算后期可以优化，左右分开去轮廓，按照上述方法计算测点
       * ***************************************/
#if 0
      /*打印vector*/
      for(vector<TEST_LOCAL>::iterator it=local_array.begin();it!=local_array.end();it++)
	{
		std::cout<<(*it).dist<<" "<<(*it).x<<"  "<<(*it).y<<std::endl; 
	}

#endif

    //cout << Body_Left << endl;
    //cout << Body_right << endl;
    if (judge_posture(Body_right, Body_Left, rectangle_x)) //此函数用于判断体态，若返回false则说明当前情况下的检测不可用
    {

        Body_Data.Body_Buttocks = Get_Buttocks_width(Hip_width_area, drawing);
        Body_Data.Body_Weigth = Get_Body_weigth(Shoulder_width_area, drawing);
        Body_Data.Body_Length = Bod_length;
    }

    else
    {
        cout << "this imge is  useless!" << endl;
    }

    //臀宽
    line(drawing, Point(rRect.tl().x, (rRect.br().y + rRect.tl().y) / 2), Point(rRect.br().x, (rRect.br().y + rRect.tl().y) / 2), cvScalar(0, 0, 255), 1, 1); //绘制中线
    *out = drawing;
}

bool cmp(Point A, Point B) //排序时的函数的参数以ｙ排序
{
    if (A.y != B.y)
    {
        return A.y < B.y;
    }
    else
    {
        return A.x < B.x;
    }
}

bool cmp_x(Point A, Point B) //排序时的函数的参数以ｘ排序
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

float Get_Buttocks_width(vector<Point> Buttocks, Mat img) //得到臀宽
{
    Point Buttocks_point_start; //轮廓坐标排序数组
    Point Buttocks_point_end;   //轮廓坐标排序数组
    float swap = 0;
    for (int i = 0; i < Buttocks.size(); i = i + 2)
    {
        if (Buttocks[i].y == Buttocks[i + 1].y)
        {
            int result = fabs(Buttocks[i].x - Buttocks[i + 1].x);
            if (result > swap)
            {
                swap = result;
                Buttocks_point_start = Buttocks[i];
                Buttocks_point_end = Buttocks[i + 1];
            }
        }
    }
    //cout << Buttocks_point_start << "  " << Buttocks_point_end << endl;
    line(img, Buttocks_point_start, Buttocks_point_end, cvScalar(0, 0, 255), 1, 1); //绘制中线
    return swap;
}

float Get_Body_weigth(vector<Point> Buttocks, Mat img) //获得体宽
{
    Point Body_weigth_poiint_start; //轮廓坐标排序数组
    Point Body_weigth_poiint_end;
    float swap = 0;
    for (int i = 0; i < Buttocks.size(); i = i + 2)
    {
        if (Buttocks[i].y == Buttocks[i + 1].y)
        {
            int result = fabs(Buttocks[i].x - Buttocks[i + 1].x);
            if (result > swap)
            {
                swap = result;
                Body_weigth_poiint_start = Buttocks[i];
                Body_weigth_poiint_end = Buttocks[i + 1];
            }
        }
    }
    //cout << Body_weigth_poiint_start << "  " << Body_weigth_poiint_end << endl;
    line(img, Body_weigth_poiint_start, Body_weigth_poiint_end, cvScalar(0, 0, 255), 1, 1); //绘制中线
    return swap;
}

bool judge_posture(vector<Point> _right, vector<Point> _left, int mid_value) //此函数用于判断猪的姿态　
{
    float error_r = 0;
    float error_l = 0;
    for (int i = 0; i < _right.size(); i++)
    {
        error_r += fabs(_right[i].x - mid_value);
    }
    for (int j = 0; j < _left.size(); j++)
    {
        error_l += fabs(_left[j].x - mid_value);
    }
    //cout << error_r << endl;
    //cout << error_l << endl;
    if (fabs(error_r - error_l) < 10000)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/***** 点到直线的距离:P到AB的距离*****/
//P为线外一点，AB为线段两个端点

float getDist_P2L(CvPoint pointP, CvPoint pointA, CvPoint pointB)
{
    //求直线方程
    int A = 0, B = 0, C = 0;
    A = pointA.y - pointB.y;
    B = pointB.x - pointA.x;
    C = pointA.x * pointB.y - pointA.y * pointB.x;
    //代入点到直线距离公式
    float distance = 0;
    distance = ((float)abs(A * pointP.x + B * pointP.y + C)) / ((float)sqrtf(A * A + B * B));
    return distance;
}

/***** 求两点间距离*****/
float getDistance(CvPoint pointO, CvPoint pointA)
{
    float distance;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);
    return distance;
}

bool comp_vect(const TEST_LOCAL &t1, const TEST_LOCAL &t2)
{
    return t1.dist < t2.dist;
}

bool comp_vect_tail(const TAIL_LOCAL &t1, const TAIL_LOCAL &t2)
{
    return t1.dist > t2.dist;
}

/*低头判定*/
bool HeadLow_judge(vector<Point> posturepoint, int length) //低頭判定
{
}

/*长度转化 参数1.存放转化后数据的结构体  参数2 转化前的像素结构体*/
void Coordinate_con(TRUE_SIZE *TrueSize, BODY_DATA BodyLocal)
{
    if (BodyLocal.Body_Buttocks != 0)
    {
        TrueSize->Body_Buttocks_T = BodyLocal.Body_Buttocks * LENGTH_FACTOR;
    }
    else
    {

        cout << "Body_Buttocks Invalid this time!" << endl;
        TrueSize->Body_Buttocks_T = -1;
    }
   if (BodyLocal.Body_Length!=0)
   {
       TrueSize->Body_Length_T = BodyLocal.Body_Length * LENGTH_FACTOR;
   }
    else
    {

        cout << "Body_Length Invalid this time!" << endl;
         TrueSize->Body_Length_T = -1;
    }

    if (BodyLocal.Body_Weigth != 0)
    {
       TrueSize->Body_Weigth_T = BodyLocal.Body_Weigth * LENGTH_FACTOR;
    }
    else
    {

        cout << "Body_Weigth_T Invalid this time!" << endl;
        TrueSize->Body_Weigth_T = -1;
    }
      
      if (BodyLocal.Body_Tall== -1)
    {
       TrueSize->Body_Tall_T = BodyLocal.Body_Tall; //深度无需转化，单位为mm做cm转化
    }
    else
    {
         TrueSize->Body_Tall_T = BodyLocal.Body_Tall / 100; //深度无需转化，单位为mm做cm转化
    }

    
}

#if 0
/*求外包络函数*/

void Convex(Mat src, Mat *out)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(src, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0)); //找轮廓
    double Max_Area = 0;                                                                     //保存当前轮廓最大面积
    for (int i = 0; i < contours.size(); i++)

    {
        double ConArea = contourArea(contours[i], true);
        if (fabs(Max_Area) < fabs(ConArea))
        {
            Max_Area = ConArea;
        }
    }

    /*****************************************
     * 用于面积的筛选，可以在一定取值内剔除不相关的值.
     * ************************************/
    vector<vector<Point> >::iterator iter = contours.begin();
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
        }
    }
    Mat drawing = Mat::zeros(src.size(), CV_8UC3);
    vector<vector<Point> > hull(contours.size());
    int rectangle_y = 0;
    int rectangle_x = 0;
    Point2f center; //定义圆中心坐标
    float radius;   //定义圆半径
    Rect rRect;

    for (size_t i = 0; i < contours.size(); i++)
    {
        convexHull(Mat(contours[i]), hull[i], false); //找凸包轮廓
    }

    for (int i = 0; i < contours.size(); i++)

    {
        Scalar color = Scalar(255, 0, 0); //颜色通道赋值
        //minEnclosingCircle(Mat(contours[i]), center, radius);
        drawContours(drawing, contours, i, color, 1, 8, hierarchy, 0, Point());        //绘制轮廓
        drawContours(drawing, hull, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point()); //绘制凸包
        //rectangle(drawing, boundingRect(contours.at(i)), cvScalar(0, 255, 0));  //
        rRect = boundingRect(contours.at(i));
        // cout << "向量hierarchy的第" << i << " 个元素内容为：" << endl
        //     << hierarchy[i] << endl
        //     << endl;
        // rectangle_x = (rRect.br().x + rRect.tl().x);
        rectangle_y = (rRect.br().y + rRect.tl().y) / 2;
        rectangle_x = (rRect.br().x + rRect.tl().x) / 2;                                                                                                          //赋值宽度
        line(drawing, Point((rRect.br().x + rRect.tl().x) / 2, rRect.tl().y), Point((rRect.br().x + rRect.tl().x) / 2, rRect.br().y), cvScalar(0, 0, 255), 1, 1); //绘制中线
                                                                                                                                                                  //drawContours(drawing, hullDefect, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point()); //绘制轮廓                                                                                                                                                               //cout << (rRect.br().x + rRect.tl().x) / 2 <<" "<< rRect.tl().y << endl;
    }

    vector<Point> Buttocks;
    vector<Point> Head;                 //头侧
    vector<Point> sort_point = hull[0]; //轮廓坐标排序数组
    //posture_point = contours[0];            //定义姿态判断运算的点集合
    /*特征点 */
    sort(sort_point.begin(), sort_point.end(), cmp); //以ｙ为优先级排序，从小到大
    for (int j = 0; j < sort_point.size(); j++)
    {
        if (sort_point[j].y < rectangle_y)
        {
            Buttocks.push_back(sort_point[j]);
        }
        else
        {
            Head.push_back(sort_point[j]);   
        }
    }
    float dist_temp = 0;
    int Heda_size = Head.size();
    for (int i = 0; i < Heda_size; i++)
    {
        TEST_LOCAL DIST_LOCAL;   //存放长度和坐标的结构体
        dist_temp = getDist_P2L(Point(Head[i].x, Head[i].y), Point((rRect.br().x + rRect.tl().x) / 2, rRect.tl().y), Point((rRect.br().x + rRect.tl().x) / 2, rRect.br().y)); //比较头部凸包与中线距离
        DIST_LOCAL.dist = dist_temp;
        DIST_LOCAL.x = Head[i].x;
        DIST_LOCAL.y = Head[i].y;
        local_array.push_back(DIST_LOCAL);
    }
   
     /*对结构体里面的长度进行排序，利用长度索引找到坐标*/
      sort(local_array.begin(),local_array.end(),comp_vect);   //按照从小到大排序，前两个为距离中心最大的包络点，，，绘制点

      circle(drawing, Point(local_array[0].x, local_array[0].y), 8, cvScalar(0, 0, 255));

      /*打印vector*/
      for(vector<TEST_LOCAL>::iterator it=local_array.begin();it!=local_array.end();it++)
	{
		std::cout<<(*it).dist<<" "<<(*it).x<<"  "<<(*it).y<<std::endl; 
	}
     


  


      line(drawing, Point(rRect.tl().x, (rRect.br().y + rRect.tl().y) / 2), Point(rRect.br().x, (rRect.br().y + rRect.tl().y) / 2), cvScalar(0, 0, 255), 1, 1); //绘制中线
      *out = drawing;
}
//    int nSize = my_dist_map.size();
//    for (int nindex = 1; nindex <= nSize; nindex++)
//    {
//      cout << my_dist_map[nindex] << endl;
//    }
//    my_dist_map.clear();
    //    /*对于所有的点进行排序*/
    //    for (int j = 0; j < dist.size();j++)
    //    {
    //        sort(dist.begin(), dist.end(), cmp_dst); //以x为优先级排序，从小到大
    //    }
    //    cout << dist[0] << endl;
    //    //cout << Buttocks << endl;
    //    //cout <<dist_temp << endl;
    //
    //
    //    //臀宽
#endif

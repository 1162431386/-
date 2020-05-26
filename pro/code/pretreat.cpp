/**
 * @file pretreat.cpp
 * @author  wangwenkai <qq:1162431386>
 * @brief 预处理算法实现
 * @date 2020-05-22
 * @copyright Copyright (c) 2020
 */

#include "pretreat.h"
/******************************* */
int left_l = 310;
int right_l = 420;
int up_l = 137;
int down_l = 479;
//感兴趣区域的阈值，可根据滑动条定
/************************************** */
/******************************************/
  //填充区域阈值
  int T_x=0;
  int T_y=0;
  int T_x0=50; 
  int T_y0=50;
 /**************************************/
/*
 * @brief  双目图像裁剪函数
 * @param  1：原图像 2：竖直方向的裁剪 3：水平裁剪数 4：左图 5：右图
 * @return 
 */
void Cut_img(Mat src_img, int m, int n, Mat *imgL, Mat *imgR)
{
    int t = m * n;
    int height = src_img.rows;
    int width = src_img.cols;

    int ceil_height = height / m;
    int ceil_width = width / n;
    Mat roi_img;

    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            Rect rect(j * ceil_width, i * ceil_height, ceil_width, ceil_height);
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

/*
 * @brief  得到RG分量，并进行直方图均衡化
 * @param  1：原图像 2：输出图像
 * @return 
 */
void Enhancement(Mat src_img, Mat *enhancement)
{
    Mat RG, dst;
    int g_nBilateralFilterValue = 16; //˫���˲�����ֵ(��ͨ������ȷ����
    getRG(src_img, &RG);
    //得到rg分量
    bilateralFilter(RG, dst, -1, g_nBilateralFilterValue * 2, g_nBilateralFilterValue / 2);
    //ֱ双边滤波
    *enhancement = equalizeIntensityHist(dst); //直方图均衡化
}

/*
 * @brief  得到RG分量
 * @param  1：原图像 2：输出图像
 * @return 
 */
void getRG(Mat src_img, Mat *RG) //过滤绿色
{
    Mat channel[3], img1;
    split(src_img, channel); //多通道分离
    //set blue channel to 0
    channel[1] = Mat::zeros(src_img.rows, src_img.cols, CV_8UC1); ///相当于创建一张黑色的图，每个像素的每个通道都为0,
    //merge red and green channels
    merge(channel, 3, *RG); //多通道合并
}

/*
 * @brief  直方图均衡化
 * @param  1：原图像 2：输出图像
 * @return 
 */
Mat equalizeIntensityHist(const Mat &inputImage)
{
    if (inputImage.channels() >= 3)
    {
        Mat ycrcb;

        cvtColor(inputImage, ycrcb, COLOR_BGR2YCrCb);

        vector<Mat> channels;
        split(ycrcb, channels);

        equalizeHist(channels[0], channels[0]);

        Mat result;
        merge(channels, ycrcb);

        cvtColor(ycrcb, result, COLOR_YCrCb2BGR);

        return result;
    }

    return Mat();
}

/*
 * @brief  直方图归一化
 * @param  onst Mat* images：输入图像
 *int nimages：输入图像的个数
 *const int* channels：需要统计直方图的第几通道
 *InputArray mask：掩膜，，计算掩膜内的直方图  ...Mat()
 *OutputArray hist:输出的直方图数组
 *int dims：需要统计直方图通道的个数
 *const int* histSize：指的是直方图分成多少个区间，就是 bin的个数
 *const float** ranges： 统计像素值得区间
 *bool uniform=true::是否对得到的直方图数组进行归一化处理
 *bool accumulate=false：在多个图像时，是否累计计算像素值得个数
 * @return 
 */
void Normalization(Mat srcImage, Mat *putImage)
{
    Mat dstHist;
    float hrange[] = {0, 255}; //所有的的灰度值范围
    const float *ranges[] = {hrange};
    int dims = 1;
    int size = 256;
    /*如果是彩色图像的话，传入的参数可以是 [0]， [1]， [2] 它们分别对应着通道 B， G， R。*/
    int channels = 0; //如果输入图像是灰度图，它的值就是 [0]；
    //输出的直方图dstHist   为一个一维数组[1*256]  其值为灰度值[0-256]对应点的出现频次
    calcHist(&srcImage, 1, &channels, Mat(), dstHist, dims, &size, ranges);
    //直方图绘制
    int scale = 1;
    int hist_h = scale * size, hist_w = size;
    int bin_w = hist_w / size;                                            //即bin_w=1;
    Mat dstImage(hist_h, hist_w, CV_8U, Scalar(0));                       //展示直方图的图纸的大小，颜色为黑色
                                                                          //将 dstHist 归一化到[0,dstHist.rows]
    normalize(dstHist, dstHist, 0, dstHist.rows, NORM_MINMAX, -1, Mat()); //归一化结果存在dstHist
    /*这个for循环可以遍历下归一化后的直方图统计数值，看看是不是都在指定范围内*/
    for (int i = 0; i < size; i++)
    {
        cout << cvRound(dstHist.at<float>(i)) << endl;
    }
    for (int i = 1; i < size; i++)
    {
        line(dstImage,
             Point(bin_w * (i - 1), hist_h - cvRound(dstHist.at<float>(i - 1))),
             Point(bin_w * (i), hist_h - cvRound(dstHist.at<float>(i))),
             Scalar(255, 255, 255), 2, 8, 0);
    }
    *putImage = dstHist;
    imshow("1-Dhistogram ", dstImage);
}

/*
 * @brief  ostu自动阈值
 * @param  1：原图像 2：输出图像
 * @return 
 */
int OTSU(Mat src)
{
    int col = src.cols;
    int row = src.rows;
    int threshold = 0;
    //初始化统计参数
    int nSumPix[256]; //每个像素值的数目
    float nProDis[256];
    for (int i = 0; i < 256; i++)
    {
        nSumPix[i] = 0;
        nProDis[i] = 0;
    }
    //统计灰度级中每个像素在整幅图像中的个数
    for (int i = 0; i < col; i++)
    {
        for (int j = 0; j < row; j++)
        {
            nSumPix[(int)src.at<uchar>(i, j)]++;
        }
    }
    //计算每个灰度级占图像中的概率分布
    for (int i = 0; i < 256; i++)
    {
        nProDis[i] = (float)nSumPix[i] / (col * row);
    }
    //w0前景点所占比例，u0前景点灰度均值
    float w0, w1, u0, u1, u0_temp, u1_temp, delta_temp;
    double delta_max = 0;
    for (int i = 0; i < 256; i++)
    {
        //i表示前景点，即分割阈值，j表示背景点
        //初始化相关参数
        w0 = w1 = u0_temp = u1_temp = u0 = u1 = delta_temp = 0;
        for (int j = 0; j < 256; j++)
        {
            if (j <= i)
            {
                w0 += nProDis[j];
                u0_temp += j * nProDis[j];
            }
            else
            {
                w1 += nProDis[j];
                u1_temp += j * nProDis[j];
            }
        }
        //各类平均灰度
        u0 = u0_temp / w0;
        u1 = u1_temp / w1;
        delta_temp = (float)(w0 * w1 * pow((u0 - u1), 2));
        if (delta_temp > delta_max)
        {
            delta_max = delta_temp;
            threshold = i;
        }
    }
    return threshold;
}

/*
根据ostu阈值实现二值化
*/
void Segment(Mat src_img, Mat *seg)
{
    int k;
    k = OTSU(src_img);
    threshold(src_img, *seg, k, 255, cv::THRESH_BINARY);
}

/*
调节感兴区域
*/
void adjust_img(Mat src_img, Mat *adjust_img)

{
    namedWindow("dst", WINDOW_GUI_EXPANDED);

    createTrackbar("left_l", "dst", &left_l, left_l_max);

    createTrackbar("right_l", "dst", &right_l, right_l_max);

    createTrackbar("up_l", "dst", &up_l, up_l_max);

    createTrackbar("down_l", "dst", &down_l, down_l_max);

    *adjust_img = src_img(Range(up_l, down_l), Range(left_l, right_l));
}

/*图像填充*/
Mat filledROIMat(Mat srcImage0, Rect rectROI)
{
    Mat srcImage = srcImage0.clone();
    Point2i pt1;
    Point2i pt2;
    pt1.x = rectROI.x;
    pt1.y = rectROI.y;
    pt2.x = rectROI.width;
    pt2.y = rectROI.height;
    rectangle(srcImage, pt1, pt2, cv::Scalar(0), -1);
    return srcImage;
}

/*填充阈值动态调节*/
void tianchong_img(Mat src_img, Mat *adjust_img)

{
    namedWindow("dst", WINDOW_GUI_EXPANDED);

    createTrackbar("T_x", "dst", &T_x, 600);

    createTrackbar("T_y", "dst", &T_y, 600);

    createTrackbar("T_x0", "dst", &T_x0, 600);

    createTrackbar("T_y0", "dst", &T_y0, 600);

    *adjust_img = filledROIMat(src_img, Rect(T_x, T_y,T_x0 ,T_y0));
}

/*此函数用于调节图像大小，选取目标阈值，只在调试时使用*/
void test_cut(Mat img)
{
    Mat Cut_img;
    while(1)
      {
          adjust_img(img, &Cut_img);
          imshow("Cut_img_R", Cut_img);
          if (waitKey(1)=='q')
          break;
      }
}

/*此函数用于调试时确定填充区域坐标，调试用*/
void test_tianchong(Mat img)
{
    Mat temp;
      while(1)
      {
          tianchong_img(img, &temp);
          imshow("img", temp);
          if (waitKey(1)=='q')
          break;
      }
}


/*限位栏填充函数*/
Mat Filling(Mat srcImg,Rect rectROI1,Rect rectROI2)
{
    Mat temp;
    temp=filledROIMat(srcImg, rectROI1);
    temp=filledROIMat(temp, rectROI2);
    return temp;
}
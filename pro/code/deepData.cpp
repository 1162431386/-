/**
 * @file   deepData.c
 * @author wangwenkai <qq:1162431386>
 * @brief  深度计算
 * @date   2020-05-22
 * @copyright Copyright (c) 2020
 */

#include "deepData.h"
using namespace std;
using namespace cv;
const int imageWidth = 1280; //摄像头单目的分辨率########--【需要调整参数的位置1】--#############
const int imageHeight = 720;

Size imageSize = Size(imageWidth, imageHeight);

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL; //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy; //映射表
Mat Rl, Rr, Pl, Pr, Q;			//校正旋转矩阵R，投影矩阵P, 重投影矩阵Q
Mat xyz;						//三维坐标

Point origin;			   //鼠标按下的起始点
Rect selection;			   //定义矩形选框
bool selectObject = false; //是否选择对象

Ptr<StereoBM> bm = StereoBM::create(16, 9);

//########--【以下双目的标定参数为：需要调整参数的位置2】--#############
//相机双目标定的结果与如下各参数的对应关系见：双目标定结果说明.pdf，pdf文档位于main.cpp（即本文档）同级文件夹--#############

/*左目相机标定参数------------------------
fc_left_x   0            cc_left_x
0           fc_left_y    cc_left_y
0           0            1
-----------------------------------------*/

Mat cameraMatrixL = (Mat_<double>(3, 3) << 1450.45938, 0,            579.88716,
	                                       0,          1452.62035,   376.32695,
	                                       0,          0,            1);


Mat distCoeffL = (Mat_<double>(5, 1) << 0.03569,     0.29314,     -0.00011,    -0.00491,     0.00000);
                                     //[kc_left_01,  kc_left_02,  kc_left_03,  kc_left_04,   kc_left_05]


/*��Ŀ����궨����------------------------
fc_right_x   0              cc_right_x
0            fc_right_y     cc_right_y
0            0              1
-----------------------------------------*/
Mat cameraMatrixR = (Mat_<double>(3, 3) << 1451.78149, 0,          684.04159,
	                                       0,          1453.54807, 350.52935,
	                                       0,          0,          1);


Mat distCoeffR = (Mat_<double>(5, 1) << 0.09596,      -0.42760,     -0.00378,     -0.00112,      0.00000);
                                     //[kc_right_01,  kc_right_02,  kc_right_03,  kc_right_04,   kc_right_05]


Mat T = (Mat_<double>(3, 1) << -28.11909,    0.11966,    0.21590);    //Tƽ������
							 //[T_01,        T_02,       T_03]

Mat rec = (Mat_<double>(3, 1) << -0.01688,   -0.00781,   -0.00766);   //rec��ת����
							  //[rec_01,     rec_02,     rec_03]


//########--双目的标定参数填写完毕-----------------------------------------------------------------------

Mat R; //R矩阵，用于中间计算

//--------------------------------------------------------------------------------------------------------
void GenerateFalseMap(cv::Mat &src, cv::Mat &disp) //颜色变换
{
	float max_val = 255.0f;
	float map[8][4] = {{0, 0, 0, 114}, {0, 0, 1, 185}, {1, 0, 0, 114}, {1, 0, 1, 174}, {0, 1, 0, 114}, {0, 1, 1, 185}, {1, 1, 0, 114}, {1, 1, 1, 0}};
	float sum = 0;
	for (int i = 0; i < 8; i++)
		sum += map[i][3];

	float weights[8];
	float cumsum[8];
	cumsum[0] = 0;
	for (int i = 0; i < 7; i++)
	{
		weights[i] = sum / map[i][3];
		cumsum[i + 1] = cumsum[i] + map[i][3] / sum;
	}

	int height_ = src.rows;
	int width_ = src.cols;

	for (int v = 0; v < height_; v++)
	{
		for (int u = 0; u < width_; u++)
		{

			float val = std::min(std::max(src.data[v * width_ + u] / max_val, 0.0f), 1.0f);

			int i;
			for (i = 0; i < 7; i++)
				if (val < cumsum[i + 1])
					break;

			float w = 1.0 - (val - cumsum[i]) * weights[i];
			uchar r = (uchar)((w * map[i][0] + (1.0 - w) * map[i + 1][0]) * 255.0);
			uchar g = (uchar)((w * map[i][1] + (1.0 - w) * map[i + 1][1]) * 255.0);
			uchar b = (uchar)((w * map[i][2] + (1.0 - w) * map[i + 1][2]) * 255.0);

			disp.data[v * width_ * 3 + 3 * u + 0] = b; //rgb内存连续存放
			disp.data[v * width_ * 3 + 3 * u + 1] = g;
			disp.data[v * width_ * 3 + 3 * u + 2] = r;
		}
	}
}
//--BM算法立体匹配--------------------------------------------------------------------
void stereo_match_bm(int, void *)
{
	int blockSize = 18, uniquenessRatio = 5, numDisparities = 11; //BM算法相关的参数，【需要调整参数的位置3，仅用于BM算法有效】--############

	bm->setBlockSize(2 * blockSize + 5); //SAD窗口大小，5~21之间为宜
	bm->setROI1(validROIL);				 //左右视图的有效像素区域
	bm->setROI2(validROIR);
	bm->setPreFilterCap(61);					//预处理滤波器值
	bm->setMinDisparity(32);					//最小视差，默认值为0, 可以是负值，int型
	bm->setNumDisparities(numDisparities * 16); //视差窗口，即最大视差值与最小视差值之差,16的整数倍
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(uniquenessRatio); //视差唯一性百分比,uniquenessRatio主要可以防止误匹配
	bm->setSpeckleWindowSize(100);			 //检查视差连通区域变化度的窗口大小
	bm->setSpeckleRange(32);				 //32视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零
	bm->setDisp12MaxDiff(-1);
	Mat disp;
	Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1); //显示
	bm->compute(rectifyImageL, rectifyImageR, disp); //输入图像必须为灰度图

	reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)
	xyz = xyz * 16;

	disp.convertTo(disp, CV_32F, 1.0 / 16); //除以16得到真实视差值,disp.convertTo(disp, CV_32F, 1.0 );
	normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);

	medianBlur(disp8U, disp8U, 9); //中值滤波

	Mat dispcolor(disp8U.size(), CV_8UC3);
	GenerateFalseMap(disp8U, dispcolor);

	//imshow("disparity", dispcolor);
}

void stereo_match_sgbm(int, void *) //SGBM匹配算法
{
	int mindisparity = 32;	//最小视差
	int SADWindowSize = 16; //滑动窗口的大小
	int ndisparities = 176; //最大的视差，要被16整除
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);

	int P1 = 4 * rectifyImageL.channels() * SADWindowSize * SADWindowSize;	//惩罚系数1
	int P2 = 32 * rectifyImageL.channels() * SADWindowSize * SADWindowSize; //惩罚系数2
	sgbm->setP1(P1);
	sgbm->setP2(P2);

	sgbm->setPreFilterCap(60);		 //滤波系数
	sgbm->setUniquenessRatio(30);	 //代价方程概率因子
	sgbm->setSpeckleRange(2);		 //相邻像素点的视差值浮动范围
	sgbm->setSpeckleWindowSize(200); //针对散斑滤波的窗口大小
	sgbm->setDisp12MaxDiff(1);		 //视差图的像素点检查
	//sgbm->setMode(cv::StereoSGBM::MODE_HH);

	Mat disp;
	sgbm->compute(rectifyImageL, rectifyImageR, disp);

	Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1); //显示

	reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)
	xyz = xyz * 16;

	disp.convertTo(disp, CV_32F, 1.0 / 16); //除以16得到真实视差值,disp.convertTo(disp, CV_32F, 1.0 );
	normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);

	medianBlur(disp8U, disp8U, 9); //中值滤波

	Mat dispcolor(disp8U.size(), CV_8UC3);
	GenerateFalseMap(disp8U, dispcolor);

	imshow("disparity", dispcolor);
}

static void onMouse(int event, int x, int y, int, void*)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	}

	switch (event)
	{
	case EVENT_LBUTTONDOWN:             //�����ť���µ��¼�
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		cout << origin << "in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
		cout << origin << endl;
		break;
	case EVENT_LBUTTONUP:               //�����ť�ͷŵ��¼�
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			break;
	}
}

/*该函数用于测量体高，入口参数为体高测点、*/
Vec<float, 3> Deep_data(cv::Point Body_high_measur_point, Mat img_L, Mat img_R)
{
	Rodrigues(rec, R); //Rodrigues变换
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, 0,
				  0, imageSize, &validROIL, &validROIR);
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

	//--读取图片，【需要调整参数的位置4】----------------------------------------------------------------
	rgbImageL = img_L;
	cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
	rgbImageR = img_R;
	cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);

	//imshow("ImageL Before Rectify", grayImageL);
	//imshow("ImageR Before Rectify", grayImageR);

	//--经过remap之后，左右相机的图像已经共面并且行对准----------------------------------------------
	remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	//--把校正结果显示出来---------------------------------------------------------------------------
	Mat rgbRectifyImageL, rgbRectifyImageR;
	cvtColor(rectifyImageL, rgbRectifyImageL, CV_GRAY2BGR);
	cvtColor(rectifyImageR, rgbRectifyImageR, CV_GRAY2BGR);

	/************
	imwrite("rectifyImageL.jpg", rectifyImageL);
	imwrite("rectifyImageR.jpg", rectifyImageR);

	namedWindow("ImageL After Rectify", WINDOW_NORMAL);
	imshow("ImageL After Rectify", rgbRectifyImageL);
	namedWindow("ImageR After Rectify", WINDOW_NORMAL);
	imshow("ImageR After Rectify", rgbRectifyImageR);
   ****************/

	//--显示在同一张图上-----------------------------------------------------------------------------
	Mat canvas;
	double sf;
	int w, h;
	sf = 600. / MAX(imageSize.width, imageSize.height);
	w = cvRound(imageSize.width * sf);
	h = cvRound(imageSize.height * sf);
	canvas.create(h, w * 2, CV_8UC3); //注意通道

	//--左图像画到画布上-----------------------------------------------------------------------------
	Mat canvasPart = canvas(Rect(w * 0, 0, w, h));							   //得到画布的一部分
	resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA); //把图像缩放到跟canvasPart一样大小
	Rect vroiL(cvRound(validROIL.x * sf), cvRound(validROIL.y * sf),		   //获得被截取的区域
			   cvRound(validROIL.width * sf), cvRound(validROIL.height * sf));
	//rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);

	//--右图像画到画布上-----------------------------------------------------------------------------
	canvasPart = canvas(Rect(w, 0, w, h)); //获得画布的另一部分
	resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y * sf),
			   cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
	//rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);

	//--画上对应的线条-------------------------------------------------------------------------------
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
	//namedWindow("rectified", WINDOW_NORMAL);
	//imshow("rectified", canvas);
//
	////--显示结果-------------------------------------------------------------------------------------
	//namedWindow("disparity", WINDOW_NORMAL);

	//--创建SAD窗口 Trackbar-------------------------------------------------------------------------
	//createTrackbar("BlockSize:\n", "disparity", &blockSize, 8, stereo_match);

	//--创建视差唯一性百分比窗口 Trackbar------------------------------------------------------------
	//createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match);

	//--创建视差窗口 Trackbar------------------------------------------------------------------------
	//createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);

	//--鼠标响应函数setMouseCallback(窗口名称, 鼠标回调函数, 传给回调函数的参数，一般取0)------------

	//stereo_match_sgbm(0, 0);   //--【需要调整参数的位置5】，本行调用sgbm算法，下一行调用BM算法，二选一进行距离测量。
	stereo_match_bm(0, 0);
	//setMouseCallback("disparity", onMouse, 0);
	Vec<float, 3> local_len = xyz.at<Vec3f>(Body_high_measur_point);
	return local_len;
}

/*寻找某个点周围8个邻点，返回9个点点集，这里用于寻找体高测点的集合*/
vector<Point> tall_point(Point T_point)
{
	vector<Point> test;
	test.push_back(Point(T_point.x - 1, T_point.y - 1));
	test.push_back(Point(T_point.x, T_point.y - 1));
	test.push_back(Point(T_point.x + 1, T_point.y - 1));

	test.push_back(Point(T_point.x - 1, T_point.y));
	test.push_back(Point(T_point.x, T_point.y));
	test.push_back(Point(T_point.x + 1, T_point.y));

	test.push_back(Point(T_point.x - 1, T_point.y + 1));
	test.push_back(Point(T_point.x, T_point.y + 1));
	test.push_back(Point(T_point.x + 1, T_point.y + 1));
	return test;
}

/*此函数用于处理深度数据，获得目标周围3x3的矩阵点的深度后，为保证数据稳定性，去除最大值和最小值，求均值*/
float Deal_Deep_Data(vector<Point> p, Mat img_L, Mat img_R) //这里形参必须是点集，然后进行点集求解，然后进行处理运算。
{
	Vec<float, 3> temp;
	vector<float> Data_tall_array;
	Data_tall_array.clear(); //每次使用前清空一下，安全起见
	for (int i = 0; i < p.size(); i++)
	{
		temp = Deep_data(p[i], img_L, img_R);
		Data_tall_array.push_back(temp[2]);  //temp[2]
	}
	sort(Data_tall_array.begin(), Data_tall_array.end(), cmp_data); //以ｙ为优先级排序，从小到大
	Data_tall_array.erase(Data_tall_array.begin());
	Data_tall_array.erase(Data_tall_array.begin()+7);   /*删除第一个和最后一个元素，（剔除最大最小值，然后滤波）*/

   /*错误值为160000，剔除*/
    vector<float>::iterator iter = Data_tall_array.begin();
    for (; iter != Data_tall_array.end();)
    {
        double len = (*iter);
        if (len == ERROR_LEN)
        {
            iter = Data_tall_array.erase(iter);
        }
        else
        {
            ++iter;
        }
    }
	float Length_Sum = 0;
	for (int i = 0; i <Data_tall_array.size() ; i++)
	{
		Length_Sum += Data_tall_array[i];
	}
	//cout << Length_Sum <<"  "<<Data_tall_array.size() << endl;
	if ( Data_tall_array.size()==0)
	{
		cout << "Body height test failed! return -1" << endl;
		return -1;
	}
	else
	{
		return Length_Sum / Data_tall_array.size();             //返回均值
	}
	
}


bool cmp_data(float x, float y)
{
	return x < y;
}

#include "enhancement.h"
void Enhancement(Mat src_img, Mat* enhancement)
{
	Mat RG,dst;
	int g_nBilateralFilterValue = 16;  //˫���˲�����ֵ(��ͨ������ȷ����
	getRG(src_img, &RG);
	//得到rg分量
	bilateralFilter(RG, dst, -1, g_nBilateralFilterValue * 2, g_nBilateralFilterValue / 2);
	//ֱ双边滤波
	*enhancement = equalizeIntensityHist(dst);   //直方图均衡化	
}

void getRG(Mat src_img, Mat* RG) //过滤绿色
{
	Mat channel[3],img1;
	split(src_img, channel);   //多通道分离
	//set blue channel to 0
	channel[0] = Mat::zeros(src_img.rows, src_img.cols, CV_8UC1); ///相当于创建一张黑色的图，每个像素的每个通道都为0,
	//merge red and green channels
	merge(channel, 3, *RG); //多通道合并
}

Mat equalizeIntensityHist(const Mat & inputImage)
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
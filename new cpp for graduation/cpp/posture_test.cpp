#include "posture_test.h"

int otsu_threshold(Mat gray)
{
	int img_width;
	int img_height;
	int size;
	float P1, P2;
	float m1 = 0, m2 = 0, mg = 0;
	int k_otsu;  //�����ҵ�����ֵ  
	float deltaTmp = 0, deltaMax = 0;  //��䷽��  
	int n0_k = 0;   //��(n0 +n1 + n2 +...nk)  
	int nk_255 = 0;

	int nk_X_k = 0;  //   ��ʾ(0*n0 + 1*n1 +2*n2 ...k*nk )  
	int nk_X_k2 = 0;  //  ��ʾ  

	int pixel_count[256];   //�洢0-255ÿ�����صĸ��� ��ni  
	img_width = gray.cols;
	img_height = gray.rows;
	size = img_width * img_height;


	for (int i = 0; i<256; i++)
	{
		pixel_count[i] = 0;
	}

	//��һ��ֱ��ͼ��ͳ��ÿ�����ص����   ��pi  
	for (int i = 0; i<img_height; i++)
	{
		for (int j = 0; j<img_width; j++)
		{
			pixel_count[gray.at<unsigned char>(i, j)]++;
		}
	}

	//����mg  �� mgΪ�ܵ�����ƽ���Ҷ�ֵ  
	for (int i = 0; i<256; i++)
	{
		mg += pixel_count[i] * i;
	}
	mg = mg / size;

	for (int k = 1; k<256; k++)    //��䷽��ͳ��  ��ֵk��0-255��ɸѡ����󷽲��k  
	{
		n0_k = 0;
		nk_X_k = 0;
		nk_X_k2 = 0;

		for (int i = 0; i <= k; i++)
		{
			n0_k += pixel_count[i];     //   P1 = n0_k  / size  

			nk_X_k += pixel_count[i] * i;  // m1 = nk_X_k / n0_k  

		}
		for (int j = k + 1; j<256; j++)   //����m2  
		{
			nk_X_k2 += pixel_count[j] * j;

		}

		P1 = n0_k / (float)size;
		m1 = nk_X_k / n0_k;
		P2 = 1 - P1;
		m2 = nk_X_k2 / (float)(size - n0_k);

		deltaTmp = P1*(m1 - mg)*(m1 - mg) + P2*(m2 - mg)*(m2 - mg);  //��䷽��  
		if (deltaTmp > deltaMax)
		{
			deltaMax = deltaTmp;
			k_otsu = k;
		}

	}
	return k_otsu;
}

void Segment(Mat src_img, Mat* seg)
{
	int k;
	k = otsu_threshold(src_img);
	threshold(src_img, *seg, k, 255, cv::THRESH_BINARY);
}
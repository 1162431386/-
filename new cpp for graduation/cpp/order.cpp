#include "order.h"
#include "uart.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "cut_img.h"
#include "enhancement.h"
#include "posture_test.h"
using namespace std;
using namespace cv;
unsigned char Recv_Buf[buff_size];
qdata buff[30];
Deal_Data *Deal_data=new Deal_Data;
pthread_t id1, id2; //线程id 全局
Pig_measurement End_data; //猪体尺信息
extern Mat img;

int left_l = 100;
int right_l = 420;
int up_l = 137;
int down_l = 400;

void *thread_1(void *)
{
		
}
	void *thread_2(void *)
 {
		int fd;
		fd = Uart_OpenDev(TTY_DEV); //串口初始化
		usart_config(fd);			//配置完成
		my_Uart_RecvFrame(fd);
		ofstream outfile;					  //创建一个文件操作类
		outfile.open("../object/result.txt"); //打开此目录下的txt文档，没有的话，则自动创建一个，不需要用户手创建 

		long int cnt = 0;
		char Name[30];
		Mat start_img;
		Mat imgL, imgR;				  //原始图像裁剪后图像
		Mat rectifyimgL, rectifyimgR; //校正后图像
		Mat enhancement;			  //图像增强图像
		Mat just_img;				  //处理图像
		Mat segment;				  //分割后图像
		Mat result;					  //边缘检测图
		Mat Error_img;				  // 凹区域
		sleep(5);                    //为安全考虑，保证稳定性　　保证主函数运行之后线程开始
		while (1)
		{
			if (get_data(buff, 30))
			{
				Deal_data->get_vaild_data(buff);
				Deal_data->get_name_data();
				Deal_data->get_instructi_data();
		      if(Deal_data->instructi==1)   //收到命令２ 开始处理图像，计算体尺信息
				{ 
				/*处理代码*/
		       imgR = img;
			   Cut_img(start_img, 1, 2, &imgL, &imgR);
			  //Rectify(Mat imgL, Mat imgR, Mat* rectifyimgL, Mat* rectifyimgR)  //校正图像
		       cvtColor(imgR, imgR, COLOR_BGR2GRAY); //灰度转化
			   equalizeHist(imgR, enhancement);	 //直方图均衡化
			   just_img = enhancement(Range(up_l, down_l), Range(left_l, right_l)); //此函数用于选定感性区域。运行通通过滑动条调节到合适位置并记录，修改当前值
			   Segment(just_img, &segment);	 //二值化分割
			   End_data = Get_data(segment, &result); //形态学处理和边缘检测寻找轮廓 面积筛选 最小凸包
			   sprintf(Name, "../object/%s_%d.bmp", Deal_data->name_data, cnt); //原图
			   imwrite(Name, img);
                }
			  if (Deal_data->instructi == 2) //收到１，说明开始进食，将数据写入到文本
			    { 
				      //写入数据
			    	 outfile << Name << ":" << cnt << endl; //写入数据到txt文本
			    	 cnt++;
			    }
			  if (cnt > 1048000)   //存完后释放覆盖原数据存储  防止硬盘空间溢出后终止
			    {
			    		cnt = 0;
			    }
			    /*  
				printf("valid_data:%s\n", Deal_data->valid_data);
				printf("name_data:%s\n", Deal_data->name_data);
				printf("instructi:%d\n", Deal_data->instructi);
				*/
			   }
		}
		outfile.close(); //关闭txt文本
	}


	void create_thread(void)
	{
		int res;
		res = pthread_create(&id1, NULL, thread_1, NULL);
		if (res) //线程创建失败
		{
			printf("create pthread error!\n");
			return;
		}
		res = pthread_create(&id1, NULL, thread_2, NULL); //创建一个线程
		if (res)										  //线程创建失败
		{
			printf("create pthread error!\n");
			return;
		}
	}

void  Deal_Data::get_vaild_data(const unsigned char *str)
	{
		
		for (int i = 0; i < 18; i++)
		{
			
			valid_data[i] = str[i + 1];
		}
		return ;
	}

void Deal_Data::get_name_data()
	{
		int i = 0;
		while (valid_data[i]!='\0')
         {
			name_data[i] = valid_data[i+4];
			i++;
		 }
	}
void Deal_Data::get_instructi_data()
{
	 char res[2];
	for (int i = 0; i < 2; i++)
	{
	  res[i]= valid_data[i + 2];
	}
	sscanf(res, "%d", &Deal_data->instructi);
}

Deal_Data::Deal_Data()
{
	memset(valid_data, '\0', sizeof(valid_data));
	memset(name_data, '\0', sizeof(name_data));
	instructi = 0;
}

Deal_Data::~Deal_Data()
{
	delete (this);
 }

 
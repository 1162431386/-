#include "order.h"
#include "uart.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
using namespace std;
using namespace cv;
unsigned char Recv_Buf[buff_size];
qdata buff[30];
Deal_Data *Deal_data=new Deal_Data;
pthread_t id1, id2; //线程id 全局

extern Mat img;

	void *thread_1(void *)
	{
		
	}
	void *thread_2(void *)
	{
		int fd;
		fd = Uart_OpenDev(TTY_DEV); //串口初始化
		usart_config(fd);			//配置完成
		int cnt = 0;
		char Name[30];
		char Name_error[30];
		Mat envir_gray_img;
		Mat current_gray_img;
		Mat result_gray_img;
		Mat test_img;
		while (1)
		{
			my_Uart_RecvFrame(fd);
			if (get_data(buff, 30))
			{
				Deal_data->get_vaild_data(buff);
				Deal_data->get_name_data();
				Deal_data->get_instructi_data();
				printf("instructi:%d\n", Deal_data->instructi);
				if(Deal_data->instructi==2)
				{
					get_environment();  //获取当前环境的照片
				}
				if(Deal_data->instructi==1)
				{
					Mat tmp_img = imread("../environment_img/q.bmp");
					cvtColor(tmp_img, envir_gray_img, CV_BGR2GRAY);
					cvtColor(img, current_gray_img, CV_BGR2GRAY);
					result_gray_img = current_gray_img - envir_gray_img;
					//threshold(result_gray_img, test_img, 125, 255, THRESH_BINARY);
					imshow("dwsd", result_gray_img);
					sprintf(Name, "../object/%s_%d.bmp", Deal_data->name_data, cnt);//原图
					//sprintf(Name_error, "../shichatu/%s_%d.bmp", Deal_data->name_data, cnt); //做差图
					//imwrite(Name_error, test_img);
					imwrite(Name, img);
					cnt++;
				}
				/* 
				printf("valid_data:%s\n", Deal_data->valid_data);
				printf("name_data:%s\n", Deal_data->name_data);
				printf("instructi:%d\n", Deal_data->instructi);
				*/
			}
		}
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

 void get_environment() //获得环境图
 {
	  imwrite("../environment_img/q.bmp", img);
	  printf("environment_img save success!\n");
 }
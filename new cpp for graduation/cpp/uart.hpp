#include <iostream>
#include "cmd_queue.h"
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#define buff_size  512//发送数组的大小,默认512.
#define TTY_DEV "/dev/ttyUSB0"// 串口设备,默认为tty0. 
int Uart_OpenDev(char *dev);
void Uart_SendFrame(int fd);
void Uart_RecvFrame(int fd);
void usart_config(int fd);
void my_Uart_RecvFrame(int fd);       //将接收到数组保存到队列buff
int get_data(qdata *buff, qsize buf_len);  //判断数据位长度是否匹配

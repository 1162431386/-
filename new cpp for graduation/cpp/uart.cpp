#include "uart.hpp"
using namespace std;
extern QUEUE que;
char send_Buf[buff_size] = "kaizi";
qdata rec_buff[22];
void usart_config(int fd)
{

        struct termios opt;
        tcgetattr(fd, &opt);
        tcflush(fd, TCIOFLUSH); //配置串口
    
        /* 设置波特率*/
        cfsetispeed(&opt, B115200);
        cfsetospeed(&opt, B115200);

        /*设置数据位*/
      /*opt.c_cflag &= ~CSIZE;
        opt.c_cflag |= CS8; //有需要可设置为7位

        /*设置奇偶校验*/
        /*opt.c_cflag &= ~PARENB;
         opt.c_iflag &= ~INPCK; //无奇偶校验

        /*设置停止位*/
        /* opt.c_cflag &= ~CSTOPB; //               这里。可以选择2位停止，具体配置可查资料

        /* 设置超时时间   -15s*/
        /*opt.c_cc[VTIME] = 150;
        opt.c_cc[VMIN] = 0;
*/
       opt.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG); //设置本地标志：不进行回送，关闭规范输入，关闭扩充输入字符处理，关闭终端产生的信号
       opt.c_iflag &= ~(IXON | ISTRIP);                 // 关闭输出流控制, 不剥除第8位
       opt.c_iflag |= (ICRNL | BRKINT );         // 将输入的CR转换为NL，使得输入和输出队列被刷新
       //opt.c_cflag &= ~PARENB;
       opt.c_iflag &= ~INPCK; //无奇偶校验
    // opt.c_oflag &= ~(OPOST);
       opt.c_cflag &= ~CSIZE;
       opt.c_cflag |= CS8;                              //有需要可设置为7位
       opt.c_cflag &= ~CSTOPB;                          // 设置输出标志：不执行输出处理
      // opt.c_cflag &= ~(PARODD | CSIZE);                // 关闭输入输出是奇校验 ，使用屏蔽位
       //opt.c_cflag |= (CS8 | CLOCAL | CREAD | PARENB);  //8位数据位，保证程序不会占用串口，能够从串口读取输入数据，允许输出产生奇偶信息以及输入的奇偶校验
       opt.c_cc[VMIN] = 22;                       //读取字符的最少个
       opt.c_cc[VTIME] = 1;                             //读取一个字符等待1*（1/10）s
       if (0 != tcsetattr(fd, TCSANOW, &opt))
       {
               perror("set baudrate");
        }
        tcflush(fd, TCIOFLUSH);
}

int Uart_OpenDev(char *dev)
{
        int fd = open(dev, O_RDWR | O_NOCTTY); //| O_NOCTTY | O_NDELAY | O_NONBLOCK
        if (-1 == fd)
        {
                perror("Can't Open Serial Port");
                return -1;
        }
        else
        {
                cout << "Open device success !" << endl;
                return fd;
        }
}
void Uart_SendFrame(int fd)
{
        if (0 == write(fd, send_Buf, strlen(send_Buf)))
        {
                printf("send end");
                // break;
        }
}

void my_Uart_RecvFrame(int fd) //将接收到数组保存到队列buff
{
        if (0 >= read(fd, rec_buff,22))
        {
                printf("uart receive error");
        }
        
        for (int i = 0; i < 22; i++)
        {
                queue_push(rec_buff[i]);
        }
       
}

int get_data(qdata *buff, qsize buf_len) //判断数据位长度是否匹配
{
        if (queue_find_cmd(buff, buf_len) == 23)
                return 1;
        else
                return -1;
}

/*                      
void Uart_RecvFrame(int fd)
{
        
     
      if(0>=read(fd,Recv,30)) ;
        {
		printf("error");
	} 

          printf("%s\n",Recv);

        
}
*/
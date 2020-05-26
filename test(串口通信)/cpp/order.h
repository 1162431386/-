#ifndef  _ORDER_H_
#define  _ORDER_H_
#include"uart.hpp"
void get_environment(void);
void uart_start(void);
void *thread_1(void*);
void  *thread_2 ( void*);
void create_thread(void);
class Deal_Data
{
public:
    Deal_Data();
     unsigned char valid_data[20]; //有效数据
     unsigned char name_data[20];  //图片命名数据
    volatile int instructi;    //指令数据
    void get_vaild_data(const unsigned char *str);
    void get_name_data();
    void get_instructi_data();
    ~Deal_Data();
};

#endif
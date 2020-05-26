#ifndef _CMD_QUEUE
#define _CMD_QUEUE
#define CMD_HEAD 0XEE                                                //帧头
#define CMD_TAIL 0XFFFCFFFF                                          //帧尾
#define QUEUE_MAX_SIZE   200
typedef unsigned char qdata;
typedef unsigned short qsize;
typedef struct _QUEUE {
    qsize _head;                                                       //队列头
    qsize _tail;                                                       //队列尾
    qdata _data[QUEUE_MAX_SIZE];                                       //队列数据缓存区
} QUEUE;
/*!
*  \brief  清空指令数据
*/
 void queue_reset(void);

/*!
* \brief  添加指令数据
* \detial 串口接收的数据，通过此函数放入指令队列
*  \param  _data 指令数据
*/
 void queue_push(qdata _data);

/*!
*  \brief  从指令队列中取出一条完整的指令
*  \param  cmd 指令接收缓存区
*  \param  buf_len 指令接收缓存区大小
*  \return  指令长度，0表示队列中无完整指令
*/
 qsize queue_find_cmd(qdata *cmd, qsize buf_len);

#endif


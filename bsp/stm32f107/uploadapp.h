
#ifndef __UPLOADAPP_H
#define __UPLOADAPP_H

#define POOLSIZE 4
#define AD_TIMES	128	//缓冲区能存多少次数据
#define AD_CHS		4	//通道数
//extern struct rt_mailbox emptymb;
extern struct rt_mailbox fullmb;
extern volatile unsigned short ad_dma_buf[POOLSIZE][AD_TIMES*AD_CHS];
extern struct rt_event upeve;

void netapp_start(void);
long ad_pwm(void);

#define M3AD_EVENT 1<<1
#endif


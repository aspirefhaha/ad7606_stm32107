
#ifndef __UPLOADAPP_H
#define __UPLOADAPP_H

#define POOLSIZE 4
#define AD_TIMES	128	//�������ܴ���ٴ�����
#define AD_CHS		4	//ͨ����
//extern struct rt_mailbox emptymb;
extern struct rt_mailbox fullmb;
extern volatile unsigned short ad_dma_buf[POOLSIZE][AD_TIMES*AD_CHS];
void netapp_start(void);
long ad_pwm(void);
#endif


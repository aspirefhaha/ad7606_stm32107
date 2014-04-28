#ifndef __M3ADH_
#define __M3ADH_


#define USE_ADC_DMA	1
#if USE_ADC_DMA			 
#define MEAN_TIME	8
#define M3ADC_CHANNELS	2
// ע��ADCΪ12λģ��ת������ֻ��ADCConvertedValue�ĵ�12λ��Ч
extern __IO uint16_t advalues[M3ADC_CHANNELS*MEAN_TIME];
#else
#define M3ADC_CHANNELS	1
#endif

void m3ad_init(void);
void m3ad_th(void);

#endif

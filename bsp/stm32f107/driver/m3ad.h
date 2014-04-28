#ifndef __M3ADH_
#define __M3ADH_


#define USE_ADC_DMA	1
#if USE_ADC_DMA			 
#define MEAN_TIME	8
#define M3ADC_CHANNELS	2
// 注：ADC为12位模数转换器，只有ADCConvertedValue的低12位有效
extern __IO uint16_t advalues[M3ADC_CHANNELS*MEAN_TIME];
#else
#define M3ADC_CHANNELS	1
#endif

void m3ad_init(void);
void m3ad_th(void);

#endif

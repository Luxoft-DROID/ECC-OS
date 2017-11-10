#ifndef __ADC_header
#define __ADC_header

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include <stdint.h>

#define ADC_1_Pin GPIO_PIN_1
#define ADC_1_GPIO_Port GPIOA
#define ADC_2_Pin GPIO_PIN_3
#define ADC_2_GPIO_Port GPIOA

#define I_const         1
#define V_const         125

extern void ADC_Init(void);
extern int ADC_Measure_I(void);
extern int ADC_Measure_V(void);
extern void ADC_Refresh(void);

#endif /* __ADC_header */
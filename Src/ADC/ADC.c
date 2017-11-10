#include "ADC.h"
#include "USART.h"

ADC_HandleTypeDef ADC_hadc1;

char ADC_Current[8] = {0,0,'.',0,0,0,0,'A'};
char ADC_Voltage[8] = {0,0,'.',0,0,0,0,'V'};  

void ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADC_1_Pin | ADC_2_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pins : ADC_1 | ADC_2 */
  GPIO_InitStruct.Pin = ADC_1_Pin | ADC_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**Common config 
    */
  ADC_hadc1.Instance = ADC1;
  ADC_hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  ADC_hadc1.Init.ScanConvMode = DISABLE;
  ADC_hadc1.Init.ContinuousConvMode = DISABLE;
  ADC_hadc1.Init.DiscontinuousConvMode = DISABLE;
  ADC_hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  ADC_hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  ADC_hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  ADC_hadc1.Init.NbrOfConversion = 1;
  ADC_hadc1.Init.DMAContinuousRequests = DISABLE;
  ADC_hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&ADC_hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&ADC_hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

int ADC_Measure_I(void)
{
  ADC_ChannelConfTypeDef sConfig;
  
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&ADC_hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  HAL_ADC_Start(&ADC_hadc1);
  int tmp_ADCValue = 0;
  if (HAL_ADC_PollForConversion(&ADC_hadc1, 1000000) == HAL_OK)
        {
            tmp_ADCValue = HAL_ADC_GetValue(&ADC_hadc1);
        }
  HAL_ADC_Stop(&ADC_hadc1);
  
  tmp_ADCValue = tmp_ADCValue * I_const;
  
  ADC_Current[0] = ((tmp_ADCValue / 100000) + '0');
  ADC_Current[1] = (((tmp_ADCValue / 10000) % 10)  + '0');
  ADC_Current[3] = (((tmp_ADCValue / 1000) % 10)  + '0');
  ADC_Current[4] = (((tmp_ADCValue / 100) % 10)  + '0');
  ADC_Current[5] = (((tmp_ADCValue / 10) % 10)  + '0');
  ADC_Current[6] = ((tmp_ADCValue % 10)  + '0'); 
  
  return(tmp_ADCValue);
}

int ADC_Measure_V(void)
{
  ADC_ChannelConfTypeDef sConfig;
  
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&ADC_hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  HAL_ADC_Start(&ADC_hadc1);
  int tmp_ADCValue = 0;
  if (HAL_ADC_PollForConversion(&ADC_hadc1, 1000000) == HAL_OK)
        {
            tmp_ADCValue = HAL_ADC_GetValue(&ADC_hadc1);
        }
  HAL_ADC_Stop(&ADC_hadc1);
  
  tmp_ADCValue = tmp_ADCValue * V_const;
  
  ADC_Voltage[0] = ((tmp_ADCValue / 100000) + '0');
  ADC_Voltage[1] = (((tmp_ADCValue / 10000) % 10)  + '0');
  ADC_Voltage[3] = (((tmp_ADCValue / 1000) % 10)  + '0');
  ADC_Voltage[4] = (((tmp_ADCValue / 100) % 10)  + '0');
  ADC_Voltage[5] = (((tmp_ADCValue / 10) % 10)  + '0');
  ADC_Voltage[6] = ((tmp_ADCValue % 10)  + '0'); 
  
  return(tmp_ADCValue);
}

void ADC_Refresh(void)
{
  HAL_UART_Transmit(USART_Used, (uint8_t *) "I : ", 4, 10);
  HAL_UART_Transmit(USART_Used, (uint8_t *) ADC_Current, 8, 100000);   
  HAL_UART_Transmit(USART_Used, (uint8_t *) "\r\n", 2, 10);

  HAL_UART_Transmit(USART_Used, (uint8_t *) "V : ", 4, 10);
  HAL_UART_Transmit(USART_Used, (uint8_t *) ADC_Voltage, 8, 100000);   
  HAL_UART_Transmit(USART_Used, (uint8_t *) "\r\n", 2, 10);
}
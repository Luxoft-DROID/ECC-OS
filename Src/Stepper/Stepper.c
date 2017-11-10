#include "Stepper.h"

Motor_Struct ECC_Mot[MAX_MOTORS];

void MOT_Init(void)
{
  ECC_Mot[0].MOT_E_Pin = GPIO_PIN_4;
  ECC_Mot[0].MOT_E_GPIO = GPIOA;
  ECC_Mot[0].MOT_DIR_Pin = GPIO_PIN_6;
  ECC_Mot[0].MOT_DIR_GPIO = GPIOA;
  ECC_Mot[0].MOT_STEP_Pin = GPIO_PIN_4;
  ECC_Mot[0].MOT_STEP_GPIO = GPIOC;
  ECC_Mot[0].MOT_STEP_Count = 0x00;
  
  ECC_Mot[1].MOT_E_Pin = GPIO_PIN_5;
  ECC_Mot[1].MOT_E_GPIO = GPIOA;
  ECC_Mot[1].MOT_DIR_Pin = GPIO_PIN_7;
  ECC_Mot[1].MOT_DIR_GPIO = GPIOA;
  ECC_Mot[1].MOT_STEP_Pin = GPIO_PIN_5;
  ECC_Mot[1].MOT_STEP_GPIO = GPIOC;
  ECC_Mot[1].MOT_STEP_Count = 0x00;
  
  ECC_Mot[2].MOT_E_Pin = GPIO_PIN_0;
  ECC_Mot[2].MOT_E_GPIO = GPIOB;
  ECC_Mot[2].MOT_DIR_Pin = GPIO_PIN_2;
  ECC_Mot[2].MOT_DIR_GPIO = GPIOB;
  ECC_Mot[2].MOT_STEP_Pin = GPIO_PIN_12;
  ECC_Mot[2].MOT_STEP_GPIO = GPIOF;
  ECC_Mot[2].MOT_STEP_Count = 0x00;
  
  ECC_Mot[3].MOT_E_Pin = GPIO_PIN_1;
  ECC_Mot[3].MOT_E_GPIO = GPIOB;
  ECC_Mot[3].MOT_DIR_Pin = GPIO_PIN_11;
  ECC_Mot[3].MOT_DIR_GPIO = GPIOF;
  ECC_Mot[3].MOT_STEP_Pin = GPIO_PIN_13;
  ECC_Mot[3].MOT_STEP_GPIO = GPIOF;
  ECC_Mot[3].MOT_STEP_Count = 0x00;
}

void MOT_Set_Enable(char MOT_Num, char MOT_E_Stat)
{
  if(MAX_MOTORS > MOT_Num)
  {
    if(MOT_OFF != MOT_E_Stat)
      {
        ECC_Mot[MOT_Num].MOT_E_GPIO->ODR &= ~ECC_Mot[MOT_Num].MOT_E_Pin;
      }
     else
      {
        ECC_Mot[MOT_Num].MOT_E_GPIO->ODR |= ECC_Mot[MOT_Num].MOT_E_Pin;
        ECC_Mot[MOT_Num].MOT_STEP_Count = 0x00;
      }
  }
}

void MOT_Set_Direction(char MOT_Num, char MOT_DIR_Stat)
{
  if(MAX_MOTORS > MOT_Num)
  {
    if(MOT_CW != MOT_DIR_Stat)
      {
        ECC_Mot[MOT_Num].MOT_DIR_GPIO->ODR |= ECC_Mot[MOT_Num].MOT_DIR_Pin;
      }
     else
      {
        ECC_Mot[MOT_Num].MOT_DIR_GPIO->ODR &= ~ECC_Mot[MOT_Num].MOT_DIR_Pin;
      }
  }
}

char MOT_Refresh(uint8_t *in)
{  
  char mot_numb = 0;
  char mot_enb = 0;
  char mot_direct = 0;
  
  //check pack to be done !!! or CRC to be attached!!!
  
  mot_numb = in[1] - '0';
  mot_enb = in[3] - '0';
  mot_direct = in[5] - '0';
  MOT_Set_Enable(mot_numb, mot_enb);
  if(MOT_OFF == mot_enb)
    {
      ECC_Mot[mot_numb].MOT_STEP_Count = 0x00; 
    }
  else
    {
      MOT_Set_Direction(mot_numb, mot_direct);
      ECC_Mot[mot_numb].MOT_STEP_Count = (((in[7] - '0') * 100) + ((in[8] - '0') * 10) + (in[9] - '0')); 
    }
  
  return (1);
}

void MOT_Pulse_Gen(void)
{
  for(char i=0; i < MAX_MOTORS;i++)
    {
      if((ECC_Mot[i].MOT_STEP_Count) != 0)
        {
          ECC_Mot[i].MOT_STEP_GPIO->ODR ^= ECC_Mot[i].MOT_STEP_Pin;
          
          if(MOT_F_EDGE == (0 != ((ECC_Mot[i].MOT_STEP_GPIO->ODR) & ECC_Mot[i].MOT_STEP_Pin)))
            {
              if(0 != ECC_Mot[i].MOT_STEP_Count)
                {
                  ECC_Mot[i].MOT_STEP_Count--;
                }
            }  
        }
    }
}
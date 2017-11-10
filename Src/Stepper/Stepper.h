#ifndef __Stepper_header
#define __Stepper_header

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MAX_MOTORS      4

#define MOT_CW          0
#define MOT_CCW         1

#define MOT_OFF         0
#define MOT_ON          1

#define MOT_F_EDGE      0
#define MOT_R_EDGE      1

typedef struct
{
  uint32_t              MOT_E_Pin;              /* Stepper motor ENABLE pin */
  GPIO_TypeDef*         MOT_E_GPIO;             /* Stepper motor ENABLE GPIO */
  uint32_t              MOT_DIR_Pin;            /* Stepper motor DIRECTION pin */
  GPIO_TypeDef*         MOT_DIR_GPIO;           /* Stepper motor DIRECTION GPIO */
  uint32_t              MOT_STEP_Pin;           /* Stepper motor STEP pin */
  GPIO_TypeDef*         MOT_STEP_GPIO;          /* Stepper motor STEP GPIO */
  uint32_t              MOT_STEP_Count;         /* Stepper motor Steps count */
}Motor_Struct;

extern void MOT_Init(void);
extern char MOT_Refresh(uint8_t *in);
extern void MOT_Pulse_Gen(void);

#endif /* __Stepper_header */
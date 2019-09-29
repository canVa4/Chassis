#ifndef __ccd_H
#define __ccd_H
#ifdef __cplusplus
 extern "C" {
#endif
     
#include "stm32f4xx_hal.h"
#include "main.h"     
#include "usart.h"
#include "gpio.h"
#include "tim.h"
     
#define CLK_HIGH HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 , GPIO_PIN_SET)     
#define CLK_LOW HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 , GPIO_PIN_RESET)     
#define SI_HIGH HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 , GPIO_PIN_SET)     
#define SI_LOW HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 , GPIO_PIN_RESET)       

extern uint32_t ADC_Value[10];
extern uint8_t ImageData[128];
extern uint16_t ccd_pos;
extern float ccd_pwm;
void ImageCapture();     
void StartIntegration(void) ;     
void CCD_init(void);   
int ccd_white_pos();
     
#ifdef __cplusplus
}
#endif
#endif /*__ ccd_H */
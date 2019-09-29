#ifndef __vavle_H
#define __vavle_H
#ifdef __cplusplus
extern "C" {
#endif
    
#include "stm32f4xx_hal.h"
#include "main.h"
#include "can.h"    

#define E13_HIGH HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13 , GPIO_PIN_SET)
#define E13_LOW HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13 , GPIO_PIN_RESET)
#define E14_HIGH HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14 , GPIO_PIN_SET)
#define E14_LOW HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14 , GPIO_PIN_RESET)
#define E15_HIGH HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15 , GPIO_PIN_SET)
#define E15_LOW HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15 , GPIO_PIN_RESET)
#define B12_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 , GPIO_PIN_SET)
#define B12_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 , GPIO_PIN_RESET)
#define B13_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 , GPIO_PIN_SET)
#define B13_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 , GPIO_PIN_RESET)
#define B14_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 , GPIO_PIN_SET)
#define B14_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 , GPIO_PIN_RESET)
#define B15_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15 , GPIO_PIN_SET)
#define B15_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15 , GPIO_PIN_RESET)
   
extern int upper_flag;
extern int throw_finish_flag;
extern int after_throw_continue_track_flag;
void deliver_rotate();
void deliver_back();
void craw_close();
void craw_open();
void upper_back_to_place();
void upper_thread();
void upper_init();
#ifdef __cplusplus
}
#endif
#endif /*__ vavle_H */
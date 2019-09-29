/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include "can_func.h"
#include "usart.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
typedef union{
        char ch[8];
        uint8_t ui8[8];
        uint16_t ui16[4];
        int16_t i16[4];
        int in[2];
        float fl[2];
        double df;
}can_change_msg;
   
typedef struct can_id_recive{
    uint32_t handle_button_id;//手柄按键
    uint32_t handle_rocker_id;//手柄摇杆
    uint32_t laser_left_id;
    uint32_t laser_right_id;
    uint32_t press_id;
    uint32_t ccd_id;
}Can_id_recive;
     
typedef struct can_id_send{
    uint32_t motor0_id;//电机id
    uint32_t motor1_id;
    uint32_t motor2_id;
    uint32_t show_screen_id;//显示最大速度
    uint32_t read_sensor_id;
    uint32_t craw_left_id;
    uint32_t craw_right_id;
    uint32_t vavle_id;
    uint32_t throw_id;
    uint32_t giver_id;
    
    
}Can_id_send;

typedef struct canlist{
	void (*func)(CanRxMsgTypeDef* pRxMsg);
	uint32_t ID;
}CanList;
     
extern Can_id_recive recive_id;
extern Can_id_send send_id;
extern can_change_msg can_RX_data;
extern can_change_msg can_TX_data; 
/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
extern int can_send_msg(uint32_t ID, uint8_t* data, uint32_t len);
extern int can_add_callback(uint32_t ID, void (*func)(CanRxMsgTypeDef* pRxMsg));
extern void can_init();
extern void send_double(double num,uint16_t id);
void send_u16(uint16_t num,uint16_t id);
void send_u8(uint8_t num,uint16_t id);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

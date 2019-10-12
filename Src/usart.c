/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
uint8_t USART_RX_BUFFER[110];       //自定义接收存放的数组
uint16_t USART_RX_FLAG=0;           //接收状态标志
int RxConter = 0;
int DMA_RxOK_Flag=0;
uint8_t DMAaRxBuffer[99];
char DMAUSART_RX_BUF[99];

int DMA_RxOK_Flag_vega=0;
uint8_t DMAaRxBuffer_vega[99];
char DMAUSART_RX_BUF_vega[99];
uint8_t aRxBuffer[RXBUFFERSIZE];           //hal库使用串口接收缓冲
uint8_t bRxBuffer[RXBUFFERSIZE]; 
char USART_RX_BUF[USART_REC_LEN];       //自定义接收存放的数组
uint16_t USART_RX_STA=0;                   //接收状态标志，接收到0x0d，0x0a结束
static union
{
    uint8_t data[24];
    float ActVal[6];
}posture;
void usart_exc_DMA_vega();
/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* UART4 init function */
void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* UART5 init function */
void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */
        
  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();
  
    /**UART4 GPIO Configuration    
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN UART4_MspInit 1 */
        
  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */
        
  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();
  
    /**UART5 GPIO Configuration    
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN UART5_MspInit 1 */
        
  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */
        
  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

  /* USER CODE BEGIN USART1_MspInit 1 */
        __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */
        
  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */
        
  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */
        
  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA1_Stream3;
    hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

  /* USER CODE BEGIN USART3_MspInit 1 */
        __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */
        
  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();
  
    /**UART4 GPIO Configuration    
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN UART4_MspDeInit 1 */
        
  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */
        
  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();
  
    /**UART5 GPIO Configuration    
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

  /* USER CODE BEGIN UART5_MspDeInit 1 */
        
  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */
        
  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */
        
  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */
        
  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */
        
  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */
        
  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */
        
  /* USER CODE END USART3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
char uart_buffer[100 + 1];
void uprintf(UART_HandleTypeDef *huart,char *fmt, ...)
{
    int size;
    
    va_list arg_ptr;
    
    va_start(arg_ptr, fmt);  
    
    size=vsnprintf(uart_buffer, 100 + 1, fmt, arg_ptr);
    va_end(arg_ptr);
    
    //HAL_UART_Transmit_DMA(huart, (uint8_t *)uart_buffer, size);
    
    HAL_UART_Transmit(huart,(uint8_t *)uart_buffer,size,1000);
}
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
if(huart->Instance==USART3)
{ 
if((USART_RX_STA&0x8000)==0)//USART_RX_STA的bit15为1，即2^15=32768,十六进制表示为0x8000，位与为0说明第十五位不为1，接收未完成
{
if((USART_RX_STA&0x4000)!=0)//USART_RX_STA的bit14为1，即2^14=16384,十六进制表示为0X4000，位与不为0说明第十四位为1，接收到0x0d
{
if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//接收到了0x0d但是缓冲区不是0x0a，则不符合我们的协议，接受错误重新开始
                else USART_RX_STA|=0x8000;//如果接收成功把USART_RX_STA按位或0x8000，把他的第15位置1表示接受完成
            }
            else// 未接收到0x0d
{
if(aRxBuffer[0]==0x0d)//缓冲区已经是0x0d的话
{
USART_RX_STA|=0x4000;//把USART_RX_STA的14位置1表示接收到0x0d
USART_RX_BUF[USART_RX_STA&0x3fff]=aRxBuffer[0];
                }
                else//如果缓冲区还不是0x0d
{
USART_RX_BUF[USART_RX_STA&0x3fff]=aRxBuffer[0];//存储数组的索引为USART_RX_STA的0~13位，所以取出他们就位与001111111111，然后把缓冲区的值赋值给存储数组
USART_RX_STA++;//存储一次标志加1，用于后面给出接收字符长度
if(USART_RX_STA>(USART_REC_LEN-1))//宏定义接收最大长度USART_REC_LEN若接收的字符长度过长，则报错重新开始接收
USART_RX_STA=0;//重新开始接收
                }       
            }                         
        }
    }
if(huart->Instance==USART1)
{
switch(count)
{
          case 0:
if(ch==0x0d)
count++;
            else
count=0;
break;
          case 1:
if(ch==0x0a)
{
i=0;
count++;
            }
            else if(ch==0x0d);
            else
count=0;
break;
          case 2:
posture.data[i]=ch;
i++;
if(i>=24)
{
i=0;
count++;
            }
break;
          case 3:
if(ch==0x0a)
count++;
            else
count=0;
break;
          case 4:
if(ch==0x0d)
{
chassis.g_vega_angle = posture.ActVal[0];
chassis.g_vega_pos_x = posture.ActVal[3];
chassis.g_vega_pos_y = posture.ActVal[4];
chassis_update();
            }
count=0;
break;
default:
count=0;
break;
        }

    } 

HAL_UART_Receive_IT(&huart3,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);  
HAL_UART_Receive_IT(&huart1,(uint8_t *)&ch,RXBUFFERSIZE); 
}*/

char s[22]={'b','y',16,6};
void send_wave(float arg1,float arg2,float arg3,float arg4){
    
    s[2]=16;  //length
    s[3]=6;   //type
    s[20]='\r';
    s[21]='\n';
    memcpy(s+4,&arg1,sizeof(arg1));
    memcpy(s+8,&arg2,sizeof(arg1));
    memcpy(s+12,&arg3,sizeof(arg1));
    memcpy(s+16,&arg4,sizeof(arg1));
    HAL_UART_Transmit_DMA(&huart3,(uint8_t *)s, 22);
    
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    
    uint32_t isrflags   = READ_REG(huart->Instance->SR);//手册上有讲，清错误都要先读SR
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))!=RESET)
    {
        READ_REG(huart->Instance->DR);//PE清标志，第二步读DR
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE);//清标志
    }
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))!=RESET)
    {
        READ_REG(huart->Instance->DR);//FE清标志，第二步读DR
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
    }
    
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))!=RESET)
    {
        READ_REG(huart->Instance->DR);//NE清标志，第二步读DR
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);
    }        
    
    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))!=RESET)
    {
        READ_REG(huart->Instance->CR1);//ORE清标志，第二步读CR
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
    }
    
}
/*
void usart_exc()
{
int cmd_argc,len;
int erro_n;
if(USART_RX_STA&0x8000){//检测是否接受完成  //接受完一次指令
len=USART_RX_STA&0x3fff;//取出接收的长度
if(len == 0){
HAL_UART_Receive_IT(&huart3,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
USART_RX_STA=0;
return;
        }
erro_n = cmd_parse(USART_RX_BUF,&cmd_argc,cmd_argv);  //解析命令
if(erro_n < 0){
//打印函数执行错误信息
if(erro_n == -3){
len = 0;
HAL_UART_Receive_IT(&huart3,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
USART_RX_STA=0;
return;
            }else if(erro_n == -2){
uprintf(CMD_USART,"命令参数长度过长\r\n");
            }else if(erro_n == -1){
uprintf(CMD_USART,"命令参数过多\r\n");
            }
len = 0;
HAL_UART_Receive_IT(&huart3,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
USART_RX_STA=0;
return;
        }
erro_n = cmd_exec(cmd_argc,cmd_argv);   //执行命令
if(erro_n < 0){
if(erro_n == -2){
uprintf(CMD_USART,"未找到命令%s\r\n",cmd_argv[0]);
            }
len = 0;
HAL_UART_Receive_IT(&huart3,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
USART_RX_STA=0;
return;
        }
len = 0;
USART_RX_STA=0;
    }
}*/

void usart_exc_camera()
{
    if(USART_RX_FLAG == 1)
    {
        int start = 0;
        int find_empty = 0;
        char temp[200] = {0};
        char temp_high[200] = {0};
        int j = 0;
        int k = 0;
        for(int i = 0;i<RxConter;i++)
        {
            if(start == 1 && USART_RX_BUFFER[i] == 'm')
            {
                break;
            }
            
            if(USART_RX_BUFFER[i] == ' ')
            {
                find_empty = 1;
                continue;
            }
            
            if(start == 1)
            {
                if(find_empty == 0)
                    temp[j++] = USART_RX_BUFFER[i];
                else
                    temp_high[k++] = USART_RX_BUFFER[i];
            }
            
            if(USART_RX_BUFFER[i] == '=')
            {
                start = 1;
                //continue;
            }  
        }
        
        if(start == 0||j>4||k>5) 
        {
            memset(USART_RX_BUFFER,0,110);
            USART_RX_FLAG = 0;
            return;
        }
        
        chassis.dis_horizone_camera = atoi(temp);
        chassis.dis_vertical_camera = atoi(temp_high);
        //send_dis_2(dis_laser1,dis_vertical,998);
        //send_dis(dis_laser1,998);
        //send_dis(dis_vertical,0x10);
        memset(USART_RX_BUFFER,0,110);
        USART_RX_FLAG = 0;
        
    }
}

/*
void usart_init()
{
//    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
//    HAL_NVIC_EnableIRQ(USART2_IRQn);

HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
HAL_NVIC_EnableIRQ(USART1_IRQn);

HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
HAL_NVIC_EnableIRQ(USART3_IRQn);

//HAL_UART_Receive_IT(&huart2,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
HAL_UART_Receive_IT(&huart1,(uint8_t *)&ch,RXBUFFERSIZE);
HAL_UART_Receive_IT(&huart3,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
}*/

void usart_DMA_init()
{ 
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)&DMAaRxBuffer, 99);
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)&DMAaRxBuffer_vega, 99);
}

void HAL_UART_IDLECallback(UART_HandleTypeDef *huart)
{  
    if(huart->Instance==USART3)
    {
        uint8_t temp;
        __HAL_UART_CLEAR_IDLEFLAG(huart);   //清除函数空闲标志
        temp= huart->Instance->SR;
        temp= huart->Instance->DR;//读出串口的数据，防止在关闭DMA期间有数据进来，造成ORE错误
        UNUSED(temp);
        //temp = hdma_usart3_rx.Instance->CNDTR; 
        //huart->hdmarx->XferCpltCallback(huart->hdmarx); //调用DMA接受完毕后的回调函数，最主要的目的是要将串口的状态设置为Ready，否则无法开启下一次DMA
        HAL_UART_DMAStop(&huart3);      //停止本次DMA
        
        strcpy((char *)DMAUSART_RX_BUF,(char *)DMAaRxBuffer);
        if(DMAUSART_RX_BUF[0]!='\0')
            DMA_RxOK_Flag=1;
        
        memset(DMAaRxBuffer,0,98);
        HAL_UART_Receive_DMA(&huart3,(uint8_t *)&DMAaRxBuffer, 99);
    }
    
    if(huart->Instance==USART1)
    {
        uint8_t temp;
        __HAL_UART_CLEAR_IDLEFLAG(huart);   //清除函数空闲标志
        temp= huart->Instance->SR;
        temp= huart->Instance->DR;//读出串口的数据，防止在关闭DMA期间有数据进来，造成ORE错误
        //temp = hdma_usart3_rx.Instance->CNDTR; 
        //huart->hdmarx->XferCpltCallback(huart->hdmarx); //调用DMA接受完毕后的回调函数，最主要的目的是要将串口的状态设置为Ready，否则无法开启下一次DMA
        HAL_UART_DMAStop(&huart1);      //停止本次DMA
        UNUSED(temp);
        strcpy((char *)DMAUSART_RX_BUF_vega,(char *)DMAaRxBuffer_vega);
        if(DMAUSART_RX_BUF_vega[0]!='\0')
            DMA_RxOK_Flag_vega=1;
        
        usart_exc_DMA_vega();
        memset(DMAaRxBuffer_vega,0,98);
        HAL_UART_Receive_DMA(&huart1,(uint8_t *)&DMAaRxBuffer_vega, 99); //zx:开启DMA？
    }
}

void usart_exc_DMA()
{
    int cmd_argc;
    int erro_n;
    if(DMA_RxOK_Flag){
        erro_n = cmd_parse((char *)DMAUSART_RX_BUF,&cmd_argc,cmd_argv);  //解析命令
        erro_n = cmd_exec(cmd_argc,cmd_argv);   //执行命令
        UNUSED(erro_n);
        memset(DMAUSART_RX_BUF,0,98);
        DMA_RxOK_Flag=0;
        
    }
}

void usart_exc_DMA_vega()
{
    if(DMA_RxOK_Flag_vega)
    {
        for(int j = 0; j < 99; j++)
        {
            if(DMAaRxBuffer_vega[j] == 0x0d)
            {
                if(DMAaRxBuffer_vega[j + 1] == 0x0a && j < 73 && DMAaRxBuffer_vega[j + 26] == 0x0a && DMAaRxBuffer_vega[j + 27] == 0x0d)
                {
                    for(int k = 0; k < 24 ; k++)
                    {
                      posture.data[k] = DMAaRxBuffer_vega[j + 2 + k];
                    }
                    chassis.g_vega_angle = posture.ActVal[0];
                    chassis.g_vega_pos_x = posture.ActVal[3];
                    chassis.g_vega_pos_y = posture.ActVal[4];
                    chassis_update();
                    DMA_RxOK_Flag_vega = 0;
                    memset(DMAUSART_RX_BUF_vega,0,98);
                    return;
                }
            }
        }
        DMA_RxOK_Flag_vega = 0;
        memset(DMAUSART_RX_BUF_vega,0,98);
        
    }
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

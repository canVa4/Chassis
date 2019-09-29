/*******************************************************************************
Copyright:      2018/12/18
File name:      maxon.c
Description:    maxon������ƣ������ٶȣ�����PIϵ������������ϵ��
Author:         ����Զ��������
Version��       1.0
Data:           2018/12/18 22:36
History:        ��
*******************************************************************************/
#include "maxon.h"


//�ٶȷ���Э�飬������Ԫ��Ϊ�ڵ㣬���һ��ΪУ��λ
uint8_t sendToSetSpeed[10] = {0x55,0xAA,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00};
uint8_t sendToSetSpeedPI[10] = {0x55,0xAA,0x01,0x00,0x84,0x00,0x00,0x00,0x00,0x00};

/**maxon��������ٶ�
*���������ں� �ٶ�ֵ
*����ֵ�� ��
*/
void maxon_canset3speed(int s1,int s2,int s0)
{
    s1 = s1 * MAX_MOTOR_SPEED / 2100;
    s2 = s2 * MAX_MOTOR_SPEED / 2100;
    s0 = s0 * MAX_MOTOR_SPEED / 2100;
    if(s1 > MAX_MOTOR_SPEED) s1 = MAX_MOTOR_SPEED;
    if(s1 < -MAX_MOTOR_SPEED) s1 = -MAX_MOTOR_SPEED;
    if(s2 > MAX_MOTOR_SPEED) s2 = MAX_MOTOR_SPEED;
    if(s2 < -MAX_MOTOR_SPEED) s2 = -MAX_MOTOR_SPEED;
    if(s0 > MAX_MOTOR_SPEED) s0 = MAX_MOTOR_SPEED;
    if(s0 < -MAX_MOTOR_SPEED) s0= -MAX_MOTOR_SPEED;
    
    if(s1 < 0) s1 = MAX_MOTOR_SPEED - s1;
    if(s2 < 0) s2 = MAX_MOTOR_SPEED - s2;
    if(s0 < 0) s0 = MAX_MOTOR_SPEED - s0;
    
    can_TX_data.ui16[0] = (uint16_t)s0;
    can_TX_data.ui16[1] = (uint16_t)s1;
    can_TX_data.ui16[2] = (uint16_t)s2;
    
    can_send_msg(send_id.motor0_id, can_TX_data.ui8, 8);
}
/*
void maxon_newsetSpeed(uint32_t can_ID, int speed)
{
    uint8_t Data[6];
    Data[0] = 'S';
    if(speed < 0) Data[1] = '1';
    else Data[1] = '0';
    speed = abs(speed);
    Data[2] = speed / 1000 + '0';
    speed %= 1000;
    Data[3] = speed / 100 + '0';
    speed %= 100;
    Data[4] = speed / 10 + '0';
    speed %= 10;
    Data[5] = speed / 1 + '0';
    can_send_msg(can_ID, Data, 6);
}

void maxon_setSpeed(UART_HandleTypeDef* USARTx, int speed){
    int i;
    int check = 0;
    
    sendToSetSpeed[NUMBER] = 0x01;
    sendToSetSpeed[DATA_4] = (uint8_t)(speed>>24);
    sendToSetSpeed[DATA_3] = (uint8_t)(speed>>16);
    sendToSetSpeed[DATA_2] = (uint8_t)(speed>>8);
    sendToSetSpeed[DATA_1] = (uint8_t)(speed>>0);
    
    for(i = 0; i<9 ; i++){
        check += sendToSetSpeed[i];
    }
    sendToSetSpeed[CHECK] = (uint8_t)check; 
    
    HAL_UART_Transmit(USARTx,(uint8_t *) sendToSetSpeed,sizeof(sendToSetSpeed),1000);	
}
*/
//p:7000 i:4000

/**maxon�������KP
*���������ں� Pֵ
*����ֵ�� ��
*/
void maxon_setSpeed_p(UART_HandleTypeDef* USARTx , int p){
    int i;
    int check = 0;
    sendToSetSpeedPI[5] = 0x02;//����ƫ����
    sendToSetSpeedPI[6] = 0x02;//���ݳ���
    sendToSetSpeedPI[7] = (uint8_t)(p>>8);
    sendToSetSpeedPI[8] = (uint8_t)(p>>0);
    
    for(i = 0; i<9 ; i++){
        check += sendToSetSpeedPI[i];
    }
    sendToSetSpeedPI[CHECK] = (uint8_t)check; 
    
    HAL_UART_Transmit(USARTx,(uint8_t *) sendToSetSpeedPI,sizeof(sendToSetSpeedPI),1000);
    
}

/**maxon�������KI
*���������ں� KI
*����ֵ�� ��
*/
void maxon_setSpeed_i(UART_HandleTypeDef* USARTx , int si){
    int i;
    int check = 0;
    
    sendToSetSpeedPI[5] = 0x03;//����ƫ����
    sendToSetSpeedPI[6] = 0x02;//���ݳ���
    sendToSetSpeedPI[7] = (uint8_t)(si>>8);
    sendToSetSpeedPI[8] = (uint8_t)(si>>0);
    
    for(i = 0; i<9 ; i++){
        check += sendToSetSpeedPI[i];
    }
    sendToSetSpeedPI[CHECK] = (uint8_t)check; 
    
    HAL_UART_Transmit(USARTx,(uint8_t *) sendToSetSpeedPI,sizeof(sendToSetSpeedPI),1000);
}

/**maxon����������
*���������ں�
*����ֵ�� ��
*/
void maxon_save(UART_HandleTypeDef* USARTx){
    uint8_t saveinfo[8] = {0x55 , 0xAA , 0x01 , 0x00 , 0x03 , 0x01 ,0x00 , 0x04};
    HAL_UART_Transmit(USARTx,(uint8_t *) saveinfo,sizeof(saveinfo),1000);
}

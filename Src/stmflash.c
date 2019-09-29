#include "stmflash.h"
#include "math.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F429������
//STM32�ڲ�FLASH��д ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/1/16
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//��ȡָ����ַ����(32λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
float flash_chassis_init_x = 0, flash_chassis_init_y = 0;
float flash_saved_x_red = 0, flash_saved_y_red = 0;
float flash_saved_x_blue = 0, flash_saved_y_blue = 0;
float flash_data[4]={0,0,0,0};//�ս�flash�е�14������
u8 STMFLASH_GetFlashSector(u32 addr);

void write_prams()
{  
  uint32_t SectorError;
  uint32_t temp;
  int i;
  FLASH_EraseInitTypeDef EraseInitStruct;
  
  HAL_FLASH_Unlock();//����flash
  
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;//ѡ��ҳ�������ǿ������������ҳ����
  EraseInitStruct.Sector=STMFLASH_GetFlashSector(FLASH_SAVE_ADDR);
  EraseInitStruct.NbSectors=1;   //������ҳ��
  
  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)//���ò�������
  { 
    uprintf(CMD_USART,"erase flash fail!\r\n");
    HAL_FLASH_Lock();
    return ;
  }
  int pram_num=sizeof(flash_data)/sizeof(flash_data[0]);
  
  for(i=0;i<pram_num;++i){
    temp=*((uint32_t *)(flash_data+i));//��flash_data[i]��Ӧ��float���͵�4�ֽ��������޷������Ͷ�����
    //flash_data��flash_data[0]��ַ,flash_data+i��flash_data[i]��ַ,Ȼ��flash_data[i]��Ӧ��float��ת��Ϊ�޷�������,��ȡ���ָ��ָ��ĵ�ַ���ֵ
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_SAVE_ADDR+i*4,temp);  //��flash��д
    uprintf(CMD_USART,"write Pram[%d]Ok!\r\n",i);
  }
  HAL_FLASH_Lock();//��סflash
  uprintf(CMD_USART,"Write OK!\r\n");
}

void load_prams()
{
  int i;
  int pram_num=sizeof(flash_data)/sizeof(flash_data[0]);
  
  for(i=0;i<pram_num;++i){
    flash_data[i]=*((float *)(FLASH_SAVE_ADDR+i*4));
    uprintf(CMD_USART,"flash_data[%d]=%lf\r\n",i,flash_data[i]);
  }	
  if(isnan(flash_data[0]))
      flash_saved_x_red = -2.35;
  else
      flash_saved_x_red = flash_data[0];
  if(isnan(flash_data[1]))
      flash_saved_y_red = 4;
  else
      flash_saved_y_red = flash_data[1];
  if(isnan(flash_data[2]))
      flash_saved_x_blue = 2.24;
  else
      flash_saved_x_blue = flash_data[2];
  if(isnan(flash_data[3]))
      flash_saved_y_blue = 4.08;
  else
      flash_saved_y_blue = flash_data[3];
  
  uprintf(CMD_USART,"\r\n");
}


u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}

//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
u8 STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_SECTOR_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_SECTOR_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_SECTOR_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_SECTOR_10;   
	return FLASH_SECTOR_11;	
}

//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F(ע�⣺���16�ֽڣ�����OTP���ݿ�����������д����)
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
	FLASH_EraseInitTypeDef FlashEraseInit;
	HAL_StatusTypeDef FlashStatus=HAL_OK;
	u32 SectorError=0;
	u32 addrx=0;
	u32 endaddr=0;	
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
    
	HAL_FLASH_Unlock();             //����	
	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
    
	if(addrx<0X1FFF0000)
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			 if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;       //�������ͣ��������� 
				FlashEraseInit.Sector=STMFLASH_GetFlashSector(addrx);   //Ҫ����������
				FlashEraseInit.NbSectors=1;                             //һ��ֻ����һ������
				FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //��ѹ��Χ��VCC=2.7~3.6V֮��!!
				if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) 
				{
					break;//����������	
				}
				}else addrx+=4;
				FLASH_WaitForLastOperation(FLASH_WAITETIME);                //�ȴ��ϴβ������
		}
	}
	FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME);            //�ȴ��ϴβ������
	if(FlashStatus==HAL_OK)
	{
		 while(WriteAddr<endaddr)//д����
		 {
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddr,*pBuffer)!=HAL_OK)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		}  
	}
	HAL_FLASH_Lock();           //����
} 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(32λ)��
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}

//////////////////////////////////////////������///////////////////////////////////////////
//WriteAddr:��ʼ��ַ
//WriteData:Ҫд�������
void Test_Write(u32 WriteAddr,u32 WriteData)   	
{
	STMFLASH_Write(WriteAddr,&WriteData,1);//д��һ���� 
}

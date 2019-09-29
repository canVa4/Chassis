#include "flash.h"

/**
  * @brief  ��ȡһ���ֵ����ݣ�������ַ���������ٴ���Ŀ��ܣ���������������
  *     
  * @note	�봫���ַ��ָ�룬��ʵ��������Ч��
  * @param  faddr: ���ݵĵ�ַָ��  
  *          
  * @retval u32: ��ַ��Ӧ��flash�е�����
  */
u32 STMFLASH_ReadWord_Inc(u32* faddr)
{
    u32 temp = *(vu32*)(* faddr);
    *faddr += 4;
    return temp;
}

/**
  * @brief  ��ȡ���ֵ����ݣ�������ַ���������ٴ���Ŀ��ܣ���������������
  *     
  * @note	�봫���ַ��ָ�룬��ʵ��������Ч��
  * @param  faddr: ���ݵĵ�ַָ��  
  *          
  * @retval u16: ��ַ��Ӧ��flash�е�����
  */
u16 STMFLASH_ReadHalfWord_Inc(u32* faddr)
{
    u16 temp = *(vu32*)(* faddr);
    *faddr += 2;
    return temp;
}

/**
  * @brief  ��ȡһ���ֽڵ����ݣ�������ַ���������ٴ���Ŀ��ܣ���������������
  *     
  * @note	�봫���ַ��ָ�룬��ʵ��������Ч��
  * @param  faddr: ���ݵĵ�ַָ��  
  *          
  * @retval u16: ��ַ��Ӧ��flash�е�����
  */
u8 STMFLASH_ReadByte_Inc(u32* faddr)
{
    u8 temp = *(vu32*)(* faddr);
    *faddr += 1;
    return temp;
}

/**
  * @brief  ��ȡһ��float�����ݣ�������ַ���������ٴ���Ŀ��ܣ���������������
  *     
  * @note	�봫���ַ��ָ�룬��ʵ��������Ч��
  *
  * @param  faddr: ���ݵĵ�ַָ��  
  *          
  * @retval float: ��ַ��Ӧ��flash�е�����
  */
float STMFLASH_ReadFloat_Inc(u32* faddr)
{
    float temp = *(float*)(* faddr);
    *faddr += sizeof(float);
    return temp;
}

/**
  * @brief  ��ȡһ���ֵ�����
  *     
  * @param  faddr: ���ݵĵ�ַ 
  *          
  * @retval u32: ��ַ��Ӧ��flash�е�����
  */
u32 STMFLASH_ReadWord(u32 faddr)
{
    return *(vu32*)faddr;
}

/**
  * @brief  ��ȡһ�����ֵ�����
  *     
  * @param  faddr: ���ݵĵ�ַ 
  *          
  * @retval u16: ��ַ��Ӧ��flash�е�����
  */
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16*)faddr;
}

/**
  * @brief  ��ȡһ���ֽڵ�����
  *     
  * @param  faddr: ���ݵĵ�ַ 
  *          
  * @retval u8: ��ַ��Ӧ��flash�е�����
  */
u8 STMFLASH_ReadByte(u32 faddr)
{
    return *(vu8*)faddr;
}

/**
  * @brief  ��ȡһ��float������
  *     
  * @param  faddr: ���ݵĵ�ַ 
  *          
  * @retval float: ��ַ��Ӧ��flash�е�����
  */
float STMFLASH_ReadFloat(u32 faddr)
{
    return *(float*)faddr;
}

/**
  * @brief  ����Ӧ��λ����д��һ��float�͵����ݣ���Ҫ��������һ��u32�͵����ݴ���
  *			
  * @note	����ȡʱ��ʹ��STMFLASH_ReadFloat��STMFLASH_ReadFloat_Inc
  *
  * @param  uint32_t Address: ���ݵĵ�ַ  
  *          
  * @param  float Data: ����
  *
  * @retval FLASH_Status: flash��״̬
  */
FLASH_Status FLASH_ProgramFloat(uint32_t Address, float Data)
{
	u32 * ptr = (u32 *) &Data;
	return FLASH_ProgramWord(Address, *ptr);
}

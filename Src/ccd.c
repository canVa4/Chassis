#include "ccd.h"

uint32_t ADC_Value[10];
uint8_t ImageData[128];

#define DELAY_TIME 5

void Delay(uint32_t delay_time)
{
  TIM3->CNT=0;
  HAL_TIM_Base_Start(&htim3);
  while(TIM3->CNT < delay_time);
  HAL_TIM_Base_Stop(&htim3);
}

float adc_avg()
{
    uint32_t ad = 0;
    for(int i = 0; i < 10; i++)
    {
        ad += ADC_Value[i];
    }
    float adc_power = ad / 10;
    return adc_power;
}

void CCD_init(void)
{
  CLK_HIGH;
  SI_HIGH;
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 10);
  StartIntegration();
}
/*************************************************************************

*                           蓝宙电子工作室

*

*  函数名称：StartIntegration

*  功能说明：CCD启动程序

*  参数说明：

*  函数返回：无

*  修改时间：2012-10-20

*  备    注：

*************************************************************************/
void StartIntegration(void) {
    unsigned char i;
    SI_HIGH;            /* SI  = 1 */
    Delay(5);
    CLK_HIGH;           /* CLK = 1 */
    Delay(5);
    SI_LOW;            /* SI  = 0 */
    Delay(5);
    CLK_LOW;           /* CLK = 0 */
    for(i=0; i<127; i++) {
        Delay(10);
        CLK_HIGH;       /* CLK = 1 */
        Delay(10);
        CLK_LOW;       /* CLK = 0 */
    }
    Delay(10);
    CLK_HIGH;           /* CLK = 1 */
    Delay(10);
    CLK_LOW;           /* CLK = 0 */
}
/*************************************************************************

*                           蓝宙电子工作室

*

*  函数名称：ImageCapture

*  功能说明：CCD采样程序

*  参数说明：* ImageData   采样数组

*  函数返回：无

*  修改时间：2012-10-20

*  备    注：

*ImageData =  ad_once(ADC1, AD6a, ADC_8bit);

*************************************************************************/
void ImageCapture() {
    float adc_power;
    uint8_t i;
    SI_HIGH;            /* SI  = 1 */
    Delay(DELAY_TIME);
    CLK_HIGH;           /* CLK = 1 */
    Delay(DELAY_TIME);
    SI_LOW;            /* SI  = 0 */
    Delay(2*DELAY_TIME);
    //Delay 10us for sample the first pixel

    /**/

    /*for(i = 0; i < 250; i++) {                    //更改250，让CCD的图像看上去比较平滑，

      Delay(1) ;  //200ns                  //把该值改大或者改小达到自己满意的结果。

    }*/
    //Sampling Pixel 1
    adc_power = adc_avg();
    ImageData[0] =  (uint8_t)(adc_power / 16);//ad_once(ADC0_SE15, ADC_8bit);

    CLK_LOW;           /* CLK = 0 */
    for(i=0; i<127; i++) {

        Delay(2*DELAY_TIME);
        CLK_HIGH;       /* CLK = 1 */
        Delay(2*DELAY_TIME);

        //Sampling Pixel 2~128
       adc_power = adc_avg();
       ImageData[i + 1] =   (uint8_t)(adc_power / 16);
       CLK_LOW;       /* CLK = 0 */
    }
    Delay(2*DELAY_TIME);
    CLK_HIGH;           /* CLK = 1 */
    Delay(2*DELAY_TIME);
    CLK_LOW;           /* CLK = 0 */
   for(int i = 0; i < 128; i+=2)
    {
        uprintf(CMD_USART,"%d",ImageData[i] / 26);
    }
    uprintf(CMD_USART,"\r\n");
    int pos = ccd_white_pos();
    uprintf(CMD_USART,"%d\r\n",pos);
}

int ccd_white_pos()
{
    uint8_t max = 0;
    uint8_t min = 255;
    for(int i = 0; i < 128; i+=2)
    {
        if(ImageData[i] > max)
            max = ImageData[i];
        if(ImageData[i] <min)
            min = ImageData[i];
    }
    uint8_t threshold = (25 * max + min) / 26;
    int bgn = 0;
    int end = 127;
    if(ImageData[63] > threshold && ImageData[64] > threshold)
    {
        for(int i = 63; i >=0; i--)
        {
            if(ImageData[i] < threshold)
            {
                bgn = i;
                break;
            }
        }
        for(int i = 64; i <= 127; i++)
        {
            if(ImageData[i] < threshold)
            {
                end = i;
                break;
            }
        }
    }
    else
    {
        int i = 63, j = 64;
        while(ImageData[i]  < threshold && i >= 0) i--;
        while(ImageData[j]  < threshold && j <= 127) j++;
        if(63 - i <= j - 64)
        {
            end = i;
            while(ImageData[i]  >= threshold && i >= 0) i--;
            bgn = i;
        }
        else
        {
            bgn = j;
            while(ImageData[j]  >= threshold && j <= 127) j++;
            end = j;
        }
    }
    return (bgn + end)/2;
}
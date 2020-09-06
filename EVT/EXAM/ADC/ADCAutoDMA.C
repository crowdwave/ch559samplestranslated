/********************************** (C) COPYRIGHT *********** ********************
* File Name: ADCAutoDMA.C
* Author: WCH
* Version: V1.3
* Date: 2016/6/24
* Description: CH559 ADC automatic DMA mode sampling operation, set the sampling channel to 0 and 1 round measurement, and set the sampling number to 11
************************************************** *****************************/
#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"

#define ADCCount 30
UINT16X ADCbuf[ ADCCount] _at_ 0x0040; //store ADC sampling data
UINT8 Flag;

#pragma NOAREGS

/************************************************* ******************************
* Function Name: InitADCInterrupt()
* Description: ADC interrupt initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void InitADCInterrupt()
{
// ADC_SETUP |= bADC_IE_FIFO_OV; //Enable FIFO overflow interrupt
// ADC_SETUP |= bADC_IE_AIN7_LOW; //Enable AIN7 low level interrupt
// ADC_SETUP |= bADC_IE_ACT; //ADC complete interrupt
    IE_ADC = 1; //Enable ADC interrupt
}

/************************************************* ******************************
* Function Name: InitADCAuto()
* Description: ADC automatic sampling initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void InitADCAuto()
{
    P1_IE = 0x00; //Close other data functions of P1 port, if only some channels are used, set according to needs, otherwise it will affect the normal use of IO
    ADC_SETUP |= bADC_POWER_EN; //ADC power enable
    ADC_CK_SE |= (MASK_ADC_CK_SE & 0x0C); //Set 12 frequency division
    ADC_CTRL &= ~MASK_ADC_CYCLE;
    ADC_CTRL |= 0x0C; //Set ADC automatic sampling period
    ADC_CTRL &= ~bADC_CHANN_MOD1;
    ADC_CTRL |= bADC_CHANN_MOD0; //Automatic channel selection channel 0 and channel 1
    ADC_EX_SW |= bADC_RESOLUTION; //Sampling bits 11bit
// ADC_EX_SW &= ~bADC_RESOLUTION; //Sampling bits 10bit
    mDelayuS(100); //Ensure ADC starts normally
}

/************************************************* ******************************
* Function Name: InitADCDMA(UINT16 addr,UINT8 num)
* Description: ADC DMA initialization
* Input: UINT16 addr, DMA start address
                   UINT8 num, DMA remaining count
* Output: None
* Return: None
************************************************** *****************************/
void InitADCDMA()
{
    ADC_DMA = ADCbuf; //Set DMA start address
    ADC_DMA_CN = ADCCount; //Set DMA remaining count
    ADC_SETUP |= bADC_DMA_EN; //Enable DMA and interrupt
}

/************************************************* ******************************
* Function Name: ADCInterrupt(void)
* Description: ADC interrupt service routine
************************************************** *****************************/
void ADCInterrupt( void) interrupt INT_NO_ADC using 1 //ADC interrupt service routine, use register set 1
{
  printf("%02X ",(UINT16)ADC_STAT);
    if(ADC_STAT & bADC_IF_DMA_END) //DMA complete interrupt
    {
    Flag = 1;
    ADC_STAT |= bADC_IF_DMA_END; //Clear interrupt
    }
    if(ADC_STAT & bADC_IF_FIFO_OV) //ADC FIFO overflow interrupt
    {
    ADC_STAT |= bADC_IF_FIFO_OV; //Clear interrupt
    }
    if(ADC_STAT & bADC_IF_AIN7_LOW) //AIN7 low level interrupt
    {
    ADC_STAT |= bADC_IF_AIN7_LOW; //Clear interrupt
    }
    if(ADC_STAT & bADC_IF_ACT) //ADC complete interrupt
    {
    ADC_STAT |= bADC_IF_ACT; //Clear interrupt
    }
}

main()
{
    UINT8 i;
    mDelaymS(30); //Power-on delay, wait for the internal crystal oscillator to stabilize, must be added
// CfgFsys( ); //CH559 clock selection configuration
    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("start ...\n");
    Flag = 0;
//ADC sampling has 8 channels. In port P1, channel n corresponds to ADC_CHANN (0x01 << n), such as channel 0 is 0x01, channel 1 is 0x02 and so on
    InitADCInterrupt(); //ADC automatic sampling initialization
    InitADCAuto(); //ADC interrupt initialization
    InitADCDMA();

    EA = 1; //Enable global interrupt
    while(1)
    {
      if(Flag)
{
for(i=0;i<30;i++)
{
printf("%04X ",ADCbuf[i]); //It is recommended to discard the first sampled value
}
Flag = 0;
}
    }
}

/********************************** (C) COPYRIGHT *********** ********************
* File Name: ADCManual.C
* Author: WCH
* Version: V1.3
* Date: 2016/6/24
* Description: CH559 manual sampling ADC operation, support channel switching
************************************************** *****************************/

#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"

UINT8 ADCChannel[8] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80}; //ADC channels 1-8

// #define _INT_ 1 //ADC interrupt mode

#pragma NOAREGS


/************************************************* ******************************
* Function Name: InitADCManual()
* Description: ADC manual initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void InitADCManual()
{
    P1_IE = 0x00; //Close other data functions of P1 port, if only part of the sampling channel is used, set the rest to 1, otherwise it will affect the IO function
    ADC_SETUP |= bADC_POWER_EN; //ADC power enable
    ADC_CK_SE = 0x02; //Set frequency division
    ADC_CTRL &= ~MASK_ADC_CYCLE; //Select manual sampling
    ADC_CTRL &= ~(bADC_CHANN_MOD1 | bADC_CHANN_MOD0); //Manually select channel
    ADC_CHANN = ADCChannel[0]; //Gate channel 1
    ADC_EX_SW |= bADC_RESOLUTION; //Sampling bits 11bit
// ADC_EX_SW &= ~bADC_RESOLUTION; //Sampling bits 10bit
    mDelayuS(100); //Ensure ADC starts normally
}

/************************************************* ******************************
* Function Name: ADCChanelChange()
* Description: ADC channel switch
* Input: UINT8 Chanel
* Output: None
* Return: UINT16 ADCValue
************************************************** *****************************/
UINT16 ADCChanelChange(UINT8 Chanel)
{
  UINT16 ADCValue = 0;
ADC_CHANN = Chanel; //Switch ADC channel
// ADC_EX_SW |= bADC_RESOLUTION; //ADC acquisition bits
mDelayuS(10); //Optional, wait for the channel to switch successfully
ADC_CTRL |= bADC_SAMPLE; //Manually generate sampling pulse
mDelayuS(5);
ADC_CTRL &= ~bADC_SAMPLE;
while((ADC_STAT & bADC_IF_ACT) == 0); //Non-interrupt mode, waiting for the completion of the acquisition
ADC_STAT |= bADC_IF_ACT;
ADCValue = ADC_FIFO;
  return ADCValue; //Return sample value
}

#ifdef _INT_
/************************************************* ******************************
* Function Name: InitADCInterrupt()
* Description: ADC interrupt initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void InitADCInterrupt()
{
    ADC_SETUP |= bADC_IE_FIFO_OV; //Enable FIFO overflow interrupt
// ADC_SETUP |= bADC_IE_AIN7_LOW; //Enable AIN7 low level interrupt
    ADC_SETUP |= bADC_IE_ACT; //ADC complete interrupt
    IE_ADC = 1; //Enable ADC interrupt
}

/************************************************* ******************************
* Function Name: ADCInterrupt(void)
* Description: ADC interrupt service routine
************************************************** *****************************/
void ADCInterrupt( void) interrupt INT_NO_ADC using 1 //ADC interrupt service routine, use register set 1
{
    UINT16 ADCValue = 0;
    if(ADC_STAT & bADC_IF_ACT) //ADC complete interrupt
    {
    ADC_STAT |= bADC_IF_ACT; //Clear interrupt
    }
    ADCValue = ADC_FIFO;
printf("FIFOCnt:%02X ADC_DATA:%04X \n",(UINT16)(ADC_STAT&3),(UINT16)ADC_FIFO);
/* If you need to continue sampling, enable the following program to generate manual sampling pulse */
#if 1
    ADC_CTRL |= bADC_SAMPLE; //Manually generate sampling pulse
    mDelayuS(2);
    ADC_CTRL &= ~bADC_SAMPLE;
#endif
}
#endif

main()
{
    UINT16 ADCDat;
    UINT8 i = 0;
// CfgFsys( ); //CH559 clock selection configuration
    mDelaymS(5); //Wait for the internal crystal oscillator to stabilize

    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("start ...\n");
//ADC sampling has 8 channels. In port P1, channel n corresponds to ADC_CHANN (0x01 << n), such as channel 0 is 0x01, channel 1 is 0x02 and so on
    InitADCManual(); //ADC manual sampling initialization
ADCDat=ADC_FIFO; //After ADC is turned on, there is an invalid sample value in FIFO, take it out and discard it
printf("FIFOCnt:%02X InvalidADC_DATA:%04X \n",(UINT16)(ADC_STAT&3),(UINT16)ADCDat);
#ifdef _INT_
    InitADCInterrupt(); //ADC interrupt initialization
    EA = 1; //Enable global interrupt
To
    ADC_CTRL |= bADC_SAMPLE; //Manually generate sampling pulse
    mDelayuS(2);
    ADC_CTRL &= ~bADC_SAMPLE;
#endif
    while(1) //You can check the bADC_IF_ACT bit if you don’t use interrupt mode, 1 means manual sampling is successful
    {
#ifndef _INT_
ADC_SETUP|=bADC_POWER_EN; //Turn on ADC power
        for(i=0;i<8;i++){
          ADCDat = ADCChanelChange(ADCChannel[i]); //channel i sampling
printf("FIFOCnt:%02X ADC_DATA%X:%04X \n",(UINT16)(ADC_STAT&3),(UINT16)i,(UINT16)ADCDat);
        }
ADC_SETUP&=~bADC_POWER_EN; //Turn off ADC power
        printf("-----I am the separator------\n");
        mDelaymS(1000);
//If you sample multiple channels at the same time, you can use bADC_CHANN_ID to distinguish the channel to which the sampled value belongs after the sampling is completed
#endif
    }
}

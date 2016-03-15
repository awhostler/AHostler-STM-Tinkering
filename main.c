#include "main.h"
#include "math.h"
#include "usart.h"
#include "utils/buffer8.h"
#include "comms.h"

/* Gyro constants -------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#define PI                         (float)     3.14159265f

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

/* Gyro variables -------------------------------------------------------------*/
__IO float HeadingValue = 0.0f;
float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, Buffer[3] = {0.0f};
uint8_t Xval, Yval = 0x00;

float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;

static uint16_t __timer = 0;

void Delay(uint16_t n)
{
    __timer = n;
    while (__timer > 0);
}

int main()
{
  char inByte;
    
    
  RCC->AHBENR |= 0xFFFFFFFF;
  RCC->APB2ENR |= 0xFFFFFFFF;
  RCC->APB1ENR |= 0xFFFFFFFF;

    /* Init USART */
    initIGVCUsart();

    /* Init LEDs */
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);
    STM_EVAL_LEDInit(LED5);
    STM_EVAL_LEDInit(LED6);
    STM_EVAL_LEDInit(LED7);
    STM_EVAL_LEDInit(LED8);
    STM_EVAL_LEDInit(LED9);
    STM_EVAL_LEDInit(LED10);
    
    /* Demo Gyroscope */
    Demo_GyroConfig();
    
    /* Read Gyro Angular data */
    Demo_GyroReadAngRate(Buffer);
    
    /* Update autoreload and capture compare registers value*/
    Xval = ABS((int8_t)(Buffer[0]));
    Yval = ABS((int8_t)(Buffer[1]));
    
    if ( Xval>Yval)
    {
        if ((int8_t)Buffer[0] > 5.0f)
        {
            /* LD10 On */
            STM_EVAL_LEDOn(LED10);
        }
        if ((int8_t)Buffer[0] < -5.0f)
        {
            /* LD3 On */
            STM_EVAL_LEDOn(LED3);
        }
    }
    else
    {
        if ((int8_t)Buffer[1] < -5.0f)
        {
            /* LD6 on */
            STM_EVAL_LEDOn(LED6);
        }
        if ((int8_t)Buffer[1] > 5.0f)
        {
            /* LD7 On */
            STM_EVAL_LEDOn(LED7);
        }
    }
    

  STM_EVAL_LEDOff(LED6);
  STM_EVAL_LEDOff(LED7);

  SysTick_Config(SystemCoreClock / 1000);


  while(1)
  {
    STM_EVAL_LEDOn(LED6);
    //usartPrint("STM32\r\n");
      if(usartHaveBytes()){
      inByte = usartGet();
          if (inByte <= 'z' && inByte >= 'A'){
              STM_EVAL_LEDToggle(LED5);
          }
      usartPut(inByte);
      }
    
    
    //usartPut('1');
    Delay(100);
  }
}

/*void USART1_IRQHandler()
{
  uint16_t data;
  data = USART_ReceiveData(USART1);

  USART_SendData(USART1, data + 1);
}*/

void Timer_Decrement()
{
    if (__timer > 0)
        __timer--;
} 

void assert_param(int x)
{

}

/**
 * @brief  Configure the Mems to gyroscope application.
 * @param  None
 * @retval None
 */

void Demo_GyroConfig(void)
{
    L3GD20_InitTypeDef L3GD20_InitStructure;
    L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
    
    /* Configure Mems L3GD20 */
    L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
    L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
    L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
    L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
    L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
    L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
    L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500;
    L3GD20_Init(&L3GD20_InitStructure);
    
    L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
    L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
    L3GD20_FilterConfig(&L3GD20_FilterStructure) ;
    
    L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

/**
 * @brief  Calculate the angular Data rate Gyroscope.
 * @param  pfData : Data out pointer
 * @retval None
 */
void Demo_GyroReadAngRate (float* pfData)
{
    uint8_t tmpbuffer[6] ={0};
    int16_t RawData[3] = {0};
    uint8_t tmpreg = 0;
    float sensitivity = 0;
    int i =0;
    
    L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
    
    L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);
    
    /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
    if(!(tmpreg & 0x40))
    {
        for(i=0; i<3; i++)
        {
            RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
        }
    }
    else
    {
        for(i=0; i<3; i++)
        {
            RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
        }
    }
    
    /* Switch the sensitivity value set in the CRTL4 */
    switch(tmpreg & 0x30)
    {
        case 0x00:
            sensitivity=L3G_Sensitivity_250dps;
            break;
            
        case 0x10:
            sensitivity=L3G_Sensitivity_500dps;
            break;
            
        case 0x20:
            sensitivity=L3G_Sensitivity_2000dps;
            break;
    }
    /* divide by sensitivity */
    for(i=0; i<3; i++)
    {
        pfData[i]=(float)RawData[i]/sensitivity;
    }
}

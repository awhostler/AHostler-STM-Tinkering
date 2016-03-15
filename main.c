#include "main.h"
#include "math.h"
#include "usart.h"
#include "utils/buffer8.h"
#include "comms.h"

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

  initIGVCUsart();

  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);
  STM_EVAL_LEDInit(LED7);

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

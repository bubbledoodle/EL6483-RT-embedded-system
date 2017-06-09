#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include <stdio.h>

static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks

// SysTick Handler (Interrupt Service Routine for the System Tick interrupt)
void SysTick_Handler(void){
  msTicks++;
}

// initialize the system tick
void init_systick(void){
	SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }
}

// pause for a specified number (n) of milliseconds
void delay_ms(uint32_t n) {
  uint32_t msTicks2 = msTicks + n;
  while(msTicks < msTicks2) ;
}


void init_LED_pins()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable clock to GPIOD

  GPIOD->MODER &= ~(0x3 << (2*12)); // clear the 2 bits corresponding to pin 12
  GPIOD->MODER |= (1 << (2*12));    // set pin 12 to be general purpose output
  // to enable the other pins (13,14,15), you will need to write to the appropriate bits of the ~MODER~ register.

  GPIOD->MODER &= ~(0x3 << (2*13)); // clear the 2 bits corresponding to pin 13
  GPIOD->MODER |= (1 << (2*13));    // set pin 12 to be general purpose output

  GPIOD->MODER &= ~(0x3 << (2*14)); // clear the 2 bits corresponding to pin 14
  GPIOD->MODER |= (1 << (2*14));    // set pin 12 to be general purpose output

  GPIOD->MODER &= ~(0x3 << (2*15)); // clear the 2 bits corresponding to pin 15
  GPIOD->MODER |= (1 << (2*15));    // set pin 12 to be general purpose output
}


void init_button()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // enable clock to GPIOA

  GPIOA->MODER &= ~(0x3 << (2*0)); // clear the 2 bits corresponding to pin 0
  // if the 2 bits corresponding to pin 0 are 00, then it is in input mode
}

/*
uint32_t read_button(void)
{
    uint32_t key;

    if (GPIOA->IDR & 0x01)
        key = 1;
    else
        key = 0;

    return(key);
}
*/

// how can't I write as GPIOD -> BSRRH |= (1 << (1 + 12));
// Also next time please pay attation to BSRRH only got 16 bit;
/*
void LED_On(uint32_t i)
{
    int a = i + 12;
    GPIOD->BSRRL |= (1 << a);
}


void LED_Off(uint32_t i)
{
    int a = i +12;
    GPIOD->BSRRH |= (1 << a);
}

*/

int main(void)
{
  // initialize
  SystemInit();
  init_systick();
  init_LED_pins();
  init_button();
  int i;

  int arr[6] = {1,2,4,8,8,3};
  uint32_t n = 6;
  uint32_t index = 0;


  int y = max(arr, n, &index);

/*
        LED_On(i);
        LED_Off(i);
*/
  int a = 1;
  while (1)
    {
        int a = 0;
    }
}





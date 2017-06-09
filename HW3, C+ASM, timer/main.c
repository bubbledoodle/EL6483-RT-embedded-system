#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include <stdio.h>


static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks
static volatile uint32_t cp, cr;


void initialise_monitor_handles(void);


//define struct _TimedTask and use alias TimedTask.
typedef struct _TimedTask
{
  void (*funcp)();
  float repeat_time;
  float called_time;
} TimedTask; 


TimedTask timed_tasks[5], timed_task;
TimedTask * timed_task_ptr = &timed_task;


TimedTask *add_timed_task(void(*funcp1)(), float r_time)
{
  // there is a doubt here: is add_timed_task every time called
  // it will create a new space for timed_task and its pointer?

  timed_task.funcp = funcp1;
  timed_task.repeat_time = r_time;
  return timed_task_ptr;
}


// SysTick Handler (Interrupt Service Routine for the System Tick interrupt)
void SysTick_Handler(void)
{
  if (GPIOA->IDR & 0x01)
    {
      //press button
      cp++;
      cr = 0;
    }
  else
    {
      //release button
      cp = 0;
      cr++;
    }
  msTicks++;
}


// initialize the system tick
void init_systick(void)
{
	SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }
}


// pause for a specified number (n) of milliseconds
void delay_ms(uint32_t n)
{
  uint32_t msTicks2 = msTicks + n;
  while(msTicks < msTicks2) ;
}


void init_LED_pins()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable clock to GPIOD

  GPIOD->MODER &= ~(0x3 << (2*12)); // clear the 2 bits corresponding to pin 12
  GPIOD->MODER |= (1 << (2*12));    // set pin 12 to be general purpose output
  
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


// toggle function set
void toggle_LED0()
{
  if (GPIOD->ODR & (1 << 12)) //LED down
     GPIOD->BSRRH |= (1 << 12); 
  else
     GPIOD->BSRRL |= (1 << 12); //LED up
}

void toggle_LED1()
{
  if (GPIOD->ODR & (1 << 13)) //LED down
     GPIOD->BSRRH |= (1 << 13);
  else
     GPIOD->BSRRL |= (1 << 13); //LED up
}

void toggle_LED2()
{
  if (GPIOD->ODR & (1 << 14)) //LED down
     GPIOD->BSRRH |= (1 << 14);
  else
     GPIOD->BSRRL |= (1 << 14); //LED up
}

void toggle_LED3()
{
  if (GPIOD->ODR & (1 << 15)) //LED down
     GPIOD->BSRRH |= (1 << 15);
  else
     GPIOD->BSRRL |= (1 << 15); //LED up
}

void print_times()
{
  int a = timed_tasks[4].called_time;
  printf("current time: %d\n",a/1000);
}


int FLOOR_NUM(float num);

int main(void)
{
  // initialize
  SystemInit();
  initialise_monitor_handles();
  init_systick();
  init_LED_pins();
  init_button();

  int button_flag = 0;

  // Add time tasks and initialize its called_time
  void (*fp)();
  fp = toggle_LED0;
  timed_tasks[0] = *add_timed_task(fp, 0.5);
  timed_tasks[0].called_time = msTicks;
  
  fp = toggle_LED1;
  timed_tasks[1] = *add_timed_task(fp, 0.25);
  timed_tasks[1].called_time = msTicks;

  fp = toggle_LED2;
  timed_tasks[2] = *add_timed_task(fp, 1.0);
  timed_tasks[2].called_time = msTicks;

  fp = toggle_LED3;
  timed_tasks[3] = *add_timed_task(fp, 2.0);
  timed_tasks[3].called_time = msTicks;

  fp = print_times;
  timed_tasks[4] = *add_timed_task(fp, 5.0);
  timed_tasks[4].called_time = msTicks;

  float a = 1.2;
  int a_floor = FLOOR_NUM(a);
  printf("input float number: %f turns into its floor %d\n", a, a_floor);

  while (1)
  {
    for(int j = 0; j<5; j++)
    {
      if(msTicks == timed_tasks[j].called_time + timed_tasks[j].repeat_time * 1000)
      {
        (*timed_tasks[j].funcp)();
        timed_tasks[j].called_time = msTicks;
      }
    }

    if(cp == 250 && button_flag == 0)
    {
      printf("Button pressed\n");
      button_flag = 1;
    }

    if(cr == 250 && button_flag == 1)
    {
      printf("Button released\n");
      button_flag = 0;
    }
  }
}





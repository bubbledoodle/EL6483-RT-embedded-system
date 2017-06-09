#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include <stdio.h>
#include <math.h>

#include "accelerometers/accelerometers.h"

static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks

typedef struct _TimedTask
{
  void (*funcp)();
  float repeat_time;
  float called_time;
} TimedTask; 
TimedTask timed_task[2];

float pitch, roll;

//static volatile int shut = 0;

// SysTick Handler (Interrupt Service Routine for the System Tick interrupt)
void SysTick_Handler(void)
{
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

  for (int i=12; i<=15; i++)
  {
    GPIOD->MODER &= ~(0x3 << (2*i)); // clear the 2 bits corresponding to pin i
    GPIOD->MODER |= (1 << (2*i));    // set pin i to be general purpose output
  }
}

void LED_On(uint32_t i)
{
  GPIOD->BSRRL = 1 << (i+12);
}

void LED_Off(uint32_t i)
{
  GPIOD->BSRRH = 1 << (i+12);
}


void init_button()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // enable clock to GPIOA

  GPIOA->MODER &= ~(0x3 << (2*0)); // clear the 2 bits corresponding to pin 0
  // if the 2 bits corresponding to pin 0 are 00, then it is in input mode
}


void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
	*roll = (180.0/M_PI)*atan2(acc_y, acc_z);
	*pitch = (180.0/M_PI)*atan2(-acc_x, sqrt(acc_y*acc_y+acc_z*acc_z));
}

void initialise_monitor_handles();

int main(void)
{
  // initialize
  SystemInit();
  initialise_monitor_handles();
  init_systick();
  init_LED_pins();
  init_button();
  init_accelerometers();

  void (*fp)();
  fp = update;
  float x =0, y = 0, z = 0;
  float *avg_x = &x;
  float *avg_y = &y;
  float *avg_z = &z;
  timed_task[0].repeat_time = 20;
  timed_task[0].called_time = msTicks;
  timed_task[0].funcp = fp;

  timed_task[1].repeat_time = 1000;
  timed_task[1].called_time = msTicks;

  accel_data_ini.acc_x_raw = 0;
  accel_data_ini.acc_y_raw = 0;
  accel_data_ini.acc_z_raw = 0;
  for (int i = 0; i < 5; i++)
  {
    accel_data_window[i] = accel_data_ini;
  }

  add_timed_task(read_acc_update_buffer, 0.02);
  add_timed_task(update_gesture_recognize, 0.02);
  //add_timed_task(get_roll_and_pitch, 0.02); // this is not added??????
  add_timed_task(print_acc_value, 1);

  while (1)
	{
    if ( (msTicks - timed_task[0].called_time) >= timed_task[0].repeat_time) // 20ms has elapsed
    { 
      timed_task[0].funcp();
      timed_task[0].called_time = msTicks;
      //printf("I am here\n");
    }

    if ( (msTicks - timed_task[1].called_time) >= timed_task[1].repeat_time) // 1s has elapsed
    {
      timed_task[1].called_time = msTicks;
      printf("roll = %f, pitch = %f\n", roll, pitch);
    } 
  }
}





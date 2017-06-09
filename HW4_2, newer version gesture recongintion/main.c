#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include <stdio.h>
#include <math.h>

#include "accelerometers/accelerometers.h"

static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks

typedef struct _Status
{
  int stage1_flag, stage2_flag, stage3_flag;
  int c_s1, c_s2, c_s3;
} Status;
Status status;
Status *status_ptr = &status;

typedef struct _TimedTask
{
  void (*funcp)(float pitch, float roll, Status status);
  float repeat_time;
  float called_time;
} TimedTask; 
TimedTask timed_task;

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

Status *update_stage1(Status status)
{
  status.stage1_flag = 1;
  status.c_s1 = 0;
  printf("updated1\n");
  return status_ptr;
}
Status *update_stage2(Status status)
{
  status.stage2_flag = 1;
  status.c_s2 = 0;
  printf("updated2\n");
  return status_ptr;
}
Status *update_stage3(Status status)
{
  status.stage3_flag = 1;
  status.c_s3 = 0;
  printf("updated3\n");
  return status_ptr;
}

Status *update_nothing(Status status)
{
  return status_ptr;
}

Status (*update_fptr[4])(Status status)\
    = {update_nothing, update_stage1, update_stage2, update_stage3};

void update(float pitch, float roll, Status status)
{

  // DetectingStage1
  if (status.stage1_flag == 0 && -15<= roll && roll<=15 && -15<= pitch && pitch<=15)
    status.c_s1++;
  // DetectingStage2
  if (status.stage1_flag == 1 && 35<= roll && roll<=55 && -15<= pitch && pitch <=15 &&\
    status.stage2_flag == 0)
    status.c_s2++;
  // DetectingStage3
  if (status.stage1_flag == 1 && -15<= roll && roll <=15 && -15<= pitch && pitch <=15 &&\
    status.stage2_flag == 1 && status.stage3_flag == 0)
    status.c_s3++;

  int i = status.c_s1/50 + 2 * status.c_s2/50 +\
          3 * status.c_s3/50;
  printf("i = %d\n",i);
  status = update_fptr[i](status);
  int j = status.stage1_flag && status.stage2_flag &&\
          status.stage3_flag;

  printf("j = %d\n",j);
  if (j)
    printf("Gesture Detected!\n");
}

int main(void)
{
  // initialize
  SystemInit();
  initialise_monitor_handles();
  init_systick();
  init_LED_pins();
  init_button();
  init_accelerometers();

  void (*fp)(float pitch, float roll);
  fp = update;

  timed_task.repeat_time = 20;
  timed_task.called_time = msTicks;
  timed_task.funcp = fp;

  status.c_s1 = 0, status.c_s2 = 0, status.c_s3 = 0;
  status.stage1_flag = 0, status.stage2_flag = 0, status.stage3_flag = 0;

  while (1)
	{
    if ( (msTicks - timed_task.called_time) >= timed_task.repeat_time) // 20ms has elapsed
    {    
      float a[3]; // array of 3 floats into which accelerometer data will be read
      float pitch, roll;
      read_accelerometers(a); // read data from accelerometers (X, Y, and Z axes)
      calc_pitch_roll(a[0], a[1], a[2], &pitch, &roll);
      timed_task.funcp(pitch, roll, status);
      timed_task.called_time = msTicks;
    }
  }
}





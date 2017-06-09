#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "arm_math.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "accelerometers/accelerometers.h"
#include "temperature/temperature.h"
#include "RNG/random_number_generator.h"
#include "serial_port_usb/serial_port_usb.h"
#include "speaker/speaker.h"


static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks
static volatile uint32_t mmsTicks; // counts 0.1 ms timeTicks

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

void init_timer2()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef timer2; 
  timer2.TIM_Prescaler = 83; // counter clock frequency becomes 84000000/(83+1) = 1 MHz
  timer2.TIM_CounterMode = TIM_CounterMode_Up;
  timer2.TIM_Period = 99;
  timer2.TIM_ClockDivision = TIM_CKD_DIV1;
  timer2.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timer2);
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);
}

void TIM2_IRQHandler()
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    mmsTicks ++;
  }
}

int main(void)
{
  // initialize
  SystemInit();
  // initialise_monitor_handles(); -- not required; we are printing over USB-connected serial port
  init_systick();
  init_LED_pins();
  init_button();
  init_accelerometers(); // initialize accelerometers
  init_rng(); // initialize random number generator
  init_temperature_sensor();

  //init_serial_port_usb();
  init_timer2();

  init_speaker();  // initialize speaker (audio out)
	
	int16_t audio_sample;
	
	// the code below outputs a sine wave to the speaker
	// the frequency and loudness are changed based on pitch and roll
	float audio_freq = 1500;
	float loudness = 2500;
	float last_freq_changed_time = -1; // the time at which the frequency was changed last
	while (1)
	{
		float t = mmsTicks / 10000.0; // calculate time
		if (t > (last_freq_changed_time + 0.2)) // every 0.2 seconds
		{
      float a[3]; // array of 3 floats into which accelerometer data will be read
      read_accelerometers(a); // read data from accelerometers (X, Y, and Z axes)
			float roll, pitch;
      calc_pitch_roll(a[0], a[1], a[2], &pitch, &roll);
			audio_freq = 1500 + 8.0*pitch;
			loudness = 2500 + 40.0*roll;
			last_freq_changed_time = t;
		}
		audio_sample = (int16_t) (loudness*arm_sin_f32(audio_freq*t)); // calculate one sample for the speaker
		// the CMSIS arm_sin_f32 function from arm_math.h is typically faster than sin() from math.h
		send_to_speaker(audio_sample);	// send one audio sample to the audio output
	} 

}


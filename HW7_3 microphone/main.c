#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "arm_math.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "accelerometers/accelerometers.h"
#include "temperature/temperature.h"
#include "RNG/random_number_generator.h"
#include "serial_port_usb/serial_port_usb.h"
#include "speaker/speaker.h"
#include "microphone/microphone_functions.h"
#include "LED_PWM.h"

static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks

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
  uint32_t msTicks2 = msTicks + 10*n;
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

//uint16_t array length for sound filter input buffer
#define AUDIO_BUF_SIZE 2048
// change array length as appropriate for a specific application (e.g., 4000 for 0.25 seconds since audio sampling rate is set to 16000 Hz in microphone.h)
float audio_buffer[AUDIO_BUF_SIZE];

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

  init_LED_PWM();
  set_LED_dutycycle(0,10);
  set_LED_dutycycle(1,25);
  set_LED_dutycycle(2,50);
  set_LED_dutycycle(3,100);

	microphone_init();
	microphone_start();
	// larger buffer into which data is copied from the array pointer returned by microphone_get_data_if_ready 	
	uint32_t audio_buffer_index = 0; // index variable
	
	
	while (1)
	{
		// read one array of samples from the microphone
    uint32_t n_samples; // number of samples read in one batch from the microphone
    uint16_t *audio_samples;
    while (1)
    {
      audio_samples = microphone_get_data_if_ready(&n_samples); // try to read an array of samples from the microphone
      if (audio_samples != 0) break;       // if array of samples was read
    }
	
    // copy array of samples from microphone into our array audio_buffer
    for(int i = 0; i < n_samples; i++){
			audio_buffer[audio_buffer_index ++] = (int16_t) audio_samples[i];
		  if (audio_buffer_index >= AUDIO_BUF_SIZE) break;
    }
	
    if (audio_buffer_index >= AUDIO_BUF_SIZE)
    {
      // array of audio samples is in audio_buffer


      // reset index
			audio_buffer_index  = 0;
		}
	}

}


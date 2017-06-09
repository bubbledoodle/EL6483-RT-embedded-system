#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include <stdio.h>
#include <math.h>
#include "morse_control.h"


static void init_systick();
static void delay_ms(uint32_t n);
extern volatile uint32_t msTicks;
static volatile uint32_t ButtonPressed = 0, ButtonReleased = 0;

enum ButtonStates
{
  ButtonIsReleased,
  ButtonIsPressed
};
enum ButtonInfo
{
  NoInfo,
  Short,
  Long
};
enum Control_letter
{
  A,
  T,
  M,
  R,
  non
};

static enum Control_letter control_letter = non;
static enum ButtonInfo info_state = NoInfo;
static enum ButtonStates button_state = ButtonIsReleased;
static uint32_t button_p = 0, button_r = 0, button_info = 0;

typedef struct _MorseInfo
{
  char letter;
  int code;
}morseInfo;

extern morseInfo aplhabet[26];
extern void SetMorseTable(morseInfo aplhabet[]);
uint32_t read_button(void){
  uint32_t button_bit = GPIOA->IDR & 0x1; 
  return button_bit;
}
static int code[4] = {0,0,0,0};
static int finish_flag = 0, i = 0, find_flag = 0;


// SysTick Handler (Interrupt Service Routine for the System Tick interrupt)
void SysTick_Handler(void)
{
uint32_t b = read_button();
  if (button_state == ButtonIsReleased)  // button_state is ButtonIsReleased
  {
    if (b == 0) 
    {
      button_p = 0;
      button_r ++;
    }
    else 
    { button_p ++;
      button_r = 0;
    }
    if (button_r == 5000) 
      {
        finish_flag = 1;
        button_r = 0;
        i = 0;
      }
    if (button_p == 250)
    {
      ButtonPressed = 1;
      button_state = ButtonIsPressed;
    }
  }
  else // button_state is ButtonIsPressed
  {
    if (b == 1)
      {
        button_p ++;
        button_info = button_p;
      }
    else 
      {
        button_r ++;
        button_p = 0;
      }
    if (button_r == 250)
    {
      if (button_info >= 1500) 
        {
          info_state = Long;
          printf("long pressed\n");
        }
      else 
        {
          info_state = Short;
          printf("short pressed\n");
        }
      if (~finish_flag)
        {
          code[i] = info_state;
          i++;
        }
      if (i == 4) 
        {
          finish_flag = 1;
          i = 0;
        }
      ButtonReleased = 1;
      button_state = ButtonIsReleased;
    }
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
int bittrans(void)
{
  int CODE = (code[0]) * 1000 + (code[1]) * 100 + (code[2]) * 10 + (code[3]);
  return CODE;
}

extern void Print_accelerometers(void);
extern void Print_temperature(void);
extern void Print_time(void);
extern void Print_RNG(void);
extern void Print_error(void);
funcp function_array[5] = {Print_accelerometers, Print_temperature, Print_time, Print_RNG, Print_error};

int main(void)
{
  // initialize
  SystemInit();
  initialise_monitor_handles();
  init_systick();
  init_LED_pins();
  init_button();
  init_accelerometers(); // initialize accelerometers
  init_rng(); // initialize random number generator
  init_temperature_sensor();
  SetMorseTable(aplhabet);

  uint32_t t_prev = 0;
  while (1)
	{
    if ( (msTicks - t_prev) >= 1000) // 1 second has elapsed
    {
      t_prev = msTicks;
    }
    if (finish_flag)
    {
      int CODE = bittrans();
      //printf("code is:%d \n", CODE);
      for (int j = 0; j < N_letters; j++)
      {
        int a = aplhabet[j].code;
        if (CODE - a == 0) 
          {
            find_flag = 1;
            char b = aplhabet[j].letter;

            printf("You pressed: %c\n", b);
          }
      }
      if ( (~find_flag) & (CODE > 0) ) 
        {
          printf("Error! No such morse code found!\n");
          find_flag = 0;
        }
      switch (CODE){
        case 1200: control_letter = A; break;
        case 2000: control_letter = T; break;
        case 2200: control_letter = M; break;
        case 1210: control_letter = R; break;
        default: control_letter = non; break;
      } 
      function_array[control_letter]();
      finish_flag = 0;
      code[0] = 0, code[1] = 0, code[2] = 0, code[3] = 0;
    }
  }
}





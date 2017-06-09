
#include <stdio.h>
#include "morse_control.h"
extern volatile uint32_t msTicks;
void Print_accelerometers(void)
{
	float a[3]; // array of 3 floats into which accelerometer data will be read
    read_accelerometers(a); // read data from accelerometers (X, Y, and Z axes)
    printf("%f %f %f\n", a[0], a[1], a[2]);
    float pitch, roll;
    calc_pitch_roll(a[0], a[1], a[2], &pitch, &roll);
    printf("Pitch = %f, Roll = %f\n", pitch, roll);
}

void Print_temperature(void)
{
	float temp = read_temperature_sensor();
    printf("Temp = %f\n",temp);
}

void Print_time(void)
{
	printf("Current system time is %lu s\n", msTicks/1000);
}

void Print_RNG(void)
{
	uint32_t r = get_random_number();
    printf("Random = %lu\n",r); // print unsigned int
}

void Print_error(void)
{
	printf("Error! This letter not link to any instructions\n");
}
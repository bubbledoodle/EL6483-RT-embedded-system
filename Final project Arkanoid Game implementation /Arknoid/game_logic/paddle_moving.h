
#ifndef PADDLE_MOVING
#define PADDLE_MOVING

#include "structures.h"
#include "stm32f4xx.h"
#include "accelerometers.h"
#include <math.h>
#include <stdio.h>

#define avg_win 5
typedef struct
{
	float x;
	float y;
	float z;
}Accdata;
void Paddle_moving(paddle_t *paddle);

#endif
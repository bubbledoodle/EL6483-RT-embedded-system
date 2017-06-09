#include "paddle_moving.h"

static Accdata buffer[avg_win];
static Accdata Avg;
static int i = 0;

Accdata calc_average(Accdata *circular_buffer)
{
  Accdata temp;
  temp.x = 0;
  temp.y = 0;
  temp.z = 0;

  for (int j = 0; j < 5; j++)
  {
    temp.x += circular_buffer[j].x;
    temp.y += circular_buffer[j].y;
    temp.z += circular_buffer[j].z;
  }

  temp.x = temp.x/5;
  temp.y = temp.y/5;
  temp.z = temp.z/5;
  return (temp);
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
  *roll = (180.0/M_PI)*atan2(acc_y, acc_z);
  *pitch = (180.0/M_PI)*atan2(-acc_x, sqrt(acc_y*acc_y+acc_z*acc_z));
}

void crop(paddle_t* paddle)
{
  if (paddle->x > RIGHT_LIMIT) paddle->x = RIGHT_LIMIT;
  if (paddle->x < LEFT_LIMIT)  paddle->x = LEFT_LIMIT;
}
void Paddle_moving(paddle_t *paddle)
{

  float a[3];
  float pitch, roll;
  read_accelerometers(a);

  buffer[i].x = a[0]; buffer[i].y = a[1]; buffer[i].z = a[2]; 
  //printf("buffer: %f,%f,%f\n",buffer[i].x, buffer[i].y, buffer[i].z); 
  i++;
  if (i > 4) i = 0;

  Avg = calc_average(buffer);
  calc_pitch_roll(Avg.x, Avg.y, Avg.z, &pitch, &roll);
  //printf("Pitch = %f, Roll = %f\n", pitch, roll);
  float increment = (pitch * pitch * pitch / 7000);
  paddle->x += increment;
  crop(paddle);
  printf("increment: %f, paddle_position: %lu\n", increment, paddle->x);
}

#include "acc_avh.h"
#include "accelerometers/accelerometers.h"
#include <stdio.h>

typedef struct _AccelData{
  float acc_x_raw;
  float acc_y_raw;
  float acc_z_raw;
}AccelData;
AccelData accel_data_window[SIZE_CIRCULAR_BUFFER], accel_data_ini;
static int nw = 0;
//-------------------------------------------------------------------------
/* function need be place to TimerEvent 
  1. read_new_data
  2. get_roll_and_pitch ----> this is in different frequency. 
                              print call it less, 
                              TimerEvent call it more.
  3. print_acc_values
*/

// this function only update circular buffer.
void read_new_data(){
  float a[3];
  read_accelerometers(a);
  if (nw < SIZE_CIRCULAR_BUFFER)
    nw++;
  else
    nw = 0;

  accel_data_window[nw].acc_x_raw = a[0];
  accel_data_window[nw].acc_y_raw = a[1];
  accel_data_window[nw].acc_z_raw = a[2];
}


// this functino expect x,y,z, which are all axis average out.
void calc_average(float *avg_x, float *avg_y, float *avg_z){
  *avg_x = 0;
  *avg_y = 0;
  *avg_z = 0;
  for (int i = 0; i < SIZE_CIRCULAR_BUFFER; i++){
    *avg_x += accel_data_window[i].acc_x_raw;
    *avg_y += accel_data_window[i].acc_y_raw;
    *avg_z += accel_data_window[i].acc_z_raw;
  }
  *avg_x /= SIZE_CIRCULAR_BUFFER;
  *avg_y /= SIZE_CIRCULAR_BUFFER;
  *avg_z /= SIZE_CIRCULAR_BUFFER;
}


// this function want pitch & roll out.
void get_roll_and_pitch(float *pitch, float *roll){
  float avg_x, avg_y, avg_z;
  calc_average(&avg_x, &avg_y, &avg_z);
  calc_pitch_roll(acc_x, acc_y, acc_z, &pitch, &roll);

}


//this function only expect print out.
void print_acc_values(){
  float pitch, roll;
  get_roll_and_pitch(&pitch, &roll);
  printf("roll = &f, pitch = &f\n", roll, pitch);
}
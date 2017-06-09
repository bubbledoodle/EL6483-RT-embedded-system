#ifndef	ACC_AVG_H
#define ACC_AVG_H

#define SIZE_CIRCULAR_BUFFER 5

// in recent value & 4 old value, out average
void calc_average(float *avg_x, float *avg_y, float *avg_z);

// pitch & roll calculated
void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll);

// print out function
void print_acc_values();

// update roll and pitch
void get_roll_and_pitch(float *pitch, float *roll)

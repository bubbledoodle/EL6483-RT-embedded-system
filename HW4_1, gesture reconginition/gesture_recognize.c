#include "gesture_recognize.h"
#include <stdio.h>
#include "accelerometers/accelerometers.h"

/* file gesture_recognize.c contains:
	update functions
	function jump table
	round function
	state machine update function
*/

/* some thing magic of define variable this way:
	1. enum matches mark with number
	2. define a type is function pointer, than use this type create array. Smart!
*/
enum States{
	DetectedStage1,
	DetectedStage2,
	DetectedStage3
};
static enum States states = DetectedStage1;
static int counter = 0;
typedef void ( *function_ptr) (void);
function_ptr update_func[3] ={update_state1,update_state2,update_state3};

/* V2 new function: round function====================================================
YOU SHOULD ALSO DECLARE FUNCTION USED HERE 
i.e. calc_pitch_roll
read_accelerometers is not needed as included in header file.
*/
void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll);
int round(float roll, float pitch, float droll, float dpitch){
	float pitch_act, roll_act;
    float a[3];
    read_accelerometers(a);
    calc_pitch_roll(a[0], a[1], a[2], &pitch_act, &roll_act);
    return( (pitch <= (pitch_act + dpitch)) && (pitch >= (pitch_act - dpitch)) &&\
    	  (roll <= (roll_act + droll)) && (roll >= (roll_act - droll)) ); 
}
//====================================================================================

void update_state1(){
	int flag = round(0,0,5,5);
	if (flag){
		counter ++;
		if (counter >= 50){
			states = DetectedStage2;
			printf("Stage 1 achieved!");
			counter = 0;
		}
	}
	else{
		// reset counter if outside the angle range
		counter = 0; 
	}
}

void update_state2(){	
	int flag = round(45,0,5,5);
	if (flag){
		counter ++;
		if (counter >= 50){
			states = DetectedStage3;
			printf("Stage 2 achieved!");
			counter = 0;
		}
	}
	else{
		counter = 0; 
	}
}

void update_state3(){
	int flag = round(0,0,5,5);
	if (flag){
		counter ++;
		if (counter >= 50){
			states = DetectedStage1;
			printf("Gesture Detected!");
			counter = 0;
		}
	}
	else{
		counter = 0; 
	}
}

//here using name update_gesture_recognize, contains only update_func call
void update_gesture_recognize(){
	update_func[states];
}
#include "accelerometers/accelerometers.h"
#include "temperature/temperature.h"
#include "RNG/random_number_generator.h"
#define N_letters 26
typedef void(*funcp)(void);
volatile uint32_t msTicks;
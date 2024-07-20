#ifndef __NETWORK_EVALUATE_H__
#define __NETWORK_EVALUATE_H__

#include <math.h>
#include "stabilizer_types.h"

/*
 * since the network outputs thrust on each motor,
 * we need to define a struct which stores the values
*/
void networkEvaluate(control_t *control, const float *state_array);

#endif
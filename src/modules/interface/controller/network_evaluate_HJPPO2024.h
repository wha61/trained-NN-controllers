#ifndef __NETWORK_EVALUATE_HJPPO2024_H__
#define __NETWORK_EVALUATE_HJPPO2024_H__

#include <math.h>
#include "stabilizer_types.h"
#include "network_evaluate.h"

/*
 * since the network outputs thrust on each motor,
 * we need to define a struct which stores the values
*/
void networkEvaluateHJPPO2024(control_t *control, const float *state_array);

#endif
#ifndef __CONTROLLER_NN_H__
#define __CONTROLLER_NN_H__

#include "stabilizer_types.h"
#include "network_evaluate.h"

void controllerNNInit(void);
bool controllerNNTest(void);
void controllerNN(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

void controllerNNEnableBigQuad(void);

#endif //__CONTROLLER_NN_H__


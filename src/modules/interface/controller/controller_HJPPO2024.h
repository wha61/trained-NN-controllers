#ifndef __CONTROLLER_HJPPO2024_H__
#define __CONTROLLER_HJPPO2024_H__

#include "stabilizer_types.h"
#include "network_evaluate.h"

void controllerHJPPO2024Init(void);
bool controllerHJPPO2024Test(void);
void controllerHJPPO2024(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

void controllerHJPPO2024EnableBigQuad(void);

#endif //__CONTROLLER_HJPPO2024_H__



#include "math3d.h"
#include "stabilizer_types.h"
#include <math.h>
#include "controller_nn.h"
#include "log.h"
#include "param.h"
#include "usec_time.h"
#include "motors.h"


#define MAX_THRUST 0.15f
// PWM to thrust coefficients
#define A 2.130295e-11f
#define B 1.032633e-6f
#define C 5.484560e-4f

#define MAX_XY 1.0f
#define MAX_Z 1.0f
#define MAX_LIN_VEL_XY 3.0f
#define MAX_LIN_VEL_Z 1.0f

static bool enableBigQuad = false;

static float maxThrustFactor = 0.70f;
static bool relVel = true;
static bool relOmega = true;
static bool relXYZ = true;
static uint16_t freq = 240;

// static control_t_n control_n;
static control_t last_step_control_t;
static struct mat33 rot;
static float state_array[20];
// static float state_array[22];

static uint32_t usec_eval;

void controllerNNInit(void) {
	// last_step_control_n.thrust_0 = 0.0f;
	// last_step_control_n.thrust_1 = 0.0f;
	// last_step_control_n.thrust_2 = 0.0f;
	// last_step_control_n.thrust_3 = 0.0f;
}



bool controllerNNTest(void) {
	return true;
}


void controllerNNEnableBigQuad(void)
{
	enableBigQuad = true;
}

void controllerNN(control_t *control, 
				  const setpoint_t *setpoint, 
				  const sensorData_t *sensors, 
				  const state_t *state, 
				  const stabilizerStep_t stabilizerStep)
{
	control->controlMode = controlModeForce;
	if (!RATE_DO_EXECUTE(/*RATE_100_HZ*/freq, stabilizerStep)) {
		return;
	}

	// Orientation
	struct quat q = mkquat(state->attitudeQuaternion.x, 
						   state->attitudeQuaternion.y, 
						   state->attitudeQuaternion.z, 
						   state->attitudeQuaternion.w);
	rot = quat2rotmat(q);

	// angular velocity
	float omega_roll = radians(sensors->gyro.x);
	float omega_pitch = radians(sensors->gyro.y);
	float omega_yaw = radians(sensors->gyro.z);

	// the state vector
	state_array[0] = (state->position.x - setpoint->position.x) / MAX_XY;
	state_array[1] = (state->position.y - setpoint->position.y) / MAX_XY;
	state_array[2] = (state->position.z - setpoint->position.z) / MAX_Z;
	state_array[3] = state->attitudeQuaternion.x;
	state_array[4] = state->attitudeQuaternion.y;
	state_array[5] = state->attitudeQuaternion.z;
	state_array[6] = state->attitudeQuaternion.w;
	state_array[7] = state->attitude.roll / 180.0f;
	state_array[8] = state->attitude.pitch /  180.0f;
	state_array[9] = state->attitude.yaw /  180.0f;
	state_array[10] = state->velocity.x / MAX_LIN_VEL_XY;
	state_array[11] = state->velocity.y / MAX_LIN_VEL_XY;
	state_array[12] = state->velocity.z / MAX_LIN_VEL_Z;
	state_array[13] = omega_roll;
	state_array[14] = omega_pitch;
	state_array[15] = omega_yaw;
	state_array[16] = (float)motorsGetRatio(MOTOR_M1) / UINT16_MAX;
	state_array[17] = (float)motorsGetRatio(MOTOR_M2) / UINT16_MAX;
	state_array[18] = (float)motorsGetRatio(MOTOR_M3) / UINT16_MAX;
	state_array[19] = (float)motorsGetRatio(MOTOR_M4) / UINT16_MAX;


	// if (relVel) {
	// 	state_array[3] = state->velocity.x - setpoint->velocity.x;
	// 	state_array[4] = state->velocity.y - setpoint->velocity.y;
	// 	state_array[5] = state->velocity.z - setpoint->velocity.z;
	// } else {
	// 	state_array[3] = state->velocity.x;
	// 	state_array[4] = state->velocity.y;
	// 	state_array[5] = state->velocity.z;
	// }

	// if (relXYZ) {
	// 	// rotate pos and vel
	// 	struct vec rot_pos = mvmul(mtranspose(rot), mkvec(state_array[0], state_array[1], state_array[2]));
	// 	struct vec rot_vel = mvmul(mtranspose(rot), mkvec(state_array[3], state_array[4], state_array[5]));

	// 	state_array[0] = rot_pos.x;
	// 	state_array[1] = rot_pos.y;
	// 	state_array[2] = rot_pos.z;

	// 	state_array[3] = rot_vel.x;
	// 	state_array[4] = rot_vel.y;
	// 	state_array[5] = rot_vel.z;
	// }	

	// if (relOmega) {
	// 	state_array[15] = omega_roll - radians(setpoint->attitudeRate.roll);
	// 	state_array[16] = omega_pitch - radians(setpoint->attitudeRate.pitch);
	// 	state_array[17] = omega_yaw - radians(setpoint->attitudeRate.yaw);
	// } else {
	// 	state_array[15] = omega_roll;
	// 	state_array[16] = omega_pitch;
	// 	state_array[17] = omega_yaw;
	// }


	// run the neural neural network
	uint64_t start = usecTimestamp();
	networkEvaluate(control, state_array);
	usec_eval = (uint32_t) (usecTimestamp() - start);

	if (setpoint->mode.z == modeDisable) {
		control->normalizedForces[0] = 0.0f;
		control->normalizedForces[1] = 0.0f;
		control->normalizedForces[2] = 0.0f;
		control->normalizedForces[3] = 0.0f;
	}

	last_step_control_t = *control; // update last step_control


	// convert thrusts to directly to PWM
	// need to hack the firmware (stablizer.c and power_distribution_stock.c)
	// int PWM_0, PWM_1, PWM_2, PWM_3; 
	// thrusts2PWM(&control_n, &PWM_0, &PWM_1, &PWM_2, &PWM_3);

	// if (setpoint->mode.z == modeDisable) {
	// 	control->motorRatios[0] = 0;
	// 	control->motorRatios[1] = 0;
	// 	control->motorRatios[2] = 0;
	// 	control->motorRatios[3] = 0;
	// } else {
	// 	control->motorRatios[0] = PWM_0;
	// 	control->motorRatios[1] = PWM_1;
	// 	control->motorRatios[2] = PWM_2;
	// 	control->motorRatios[3] = PWM_3;
	// }
}

PARAM_GROUP_START(ctrlNN)
PARAM_ADD(PARAM_FLOAT, max_thrust, &maxThrustFactor)
PARAM_ADD(PARAM_UINT8, rel_vel, &relVel)
PARAM_ADD(PARAM_UINT8, rel_omega, &relOmega)
PARAM_ADD(PARAM_UINT8, rel_xyz, &relXYZ)
PARAM_ADD(PARAM_UINT16, freq, &freq)
PARAM_GROUP_STOP(ctrlNN)

LOG_GROUP_START(ctrlNN)
// LOG_ADD(LOG_FLOAT, out0, &control_n.thrust_0)
// LOG_ADD(LOG_FLOAT, out1, &control_n.thrust_1)
// LOG_ADD(LOG_FLOAT, out2, &control_n.thrust_2)
// LOG_ADD(LOG_FLOAT, out3, &control_n.thrust_3)

// LOG_ADD(LOG_FLOAT, in0, &state_array[0])
// LOG_ADD(LOG_FLOAT, in1, &state_array[1])
// LOG_ADD(LOG_FLOAT, in2, &state_array[2])

// LOG_ADD(LOG_FLOAT, in3, &state_array[3])
// LOG_ADD(LOG_FLOAT, in4, &state_array[4])
// LOG_ADD(LOG_FLOAT, in5, &state_array[5])

LOG_ADD(LOG_FLOAT, inm1, &state_array[16])
LOG_ADD(LOG_FLOAT, inm2, &state_array[17])
LOG_ADD(LOG_FLOAT, inm3, &state_array[18])
LOG_ADD(LOG_FLOAT, inm4, &state_array[19])
LOG_ADD(LOG_UINT32, usec_eval, &usec_eval)

LOG_GROUP_STOP(ctrlNN)
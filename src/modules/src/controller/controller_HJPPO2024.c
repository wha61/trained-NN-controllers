
#include "math3d.h"
#include "stabilizer_types.h"
#include <math.h>
#include "controller_HJPPO2024.h"
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
static float state_array[17];
// static float state_array[22];

static uint32_t usec_eval;

void controllerHJPPO2024Init(void) {
	// last_step_control_n.thrust_0 = 0.0f;
	// last_step_control_n.thrust_1 = 0.0f;
	// last_step_control_n.thrust_2 = 0.0f;
	// last_step_control_n.thrust_3 = 0.0f;
}



bool controllerHJPPO2024Test(void) {
	return true;
}


void controllerHJPPO2024EnableBigQuad(void)
{
	enableBigQuad = true;
}

void controllerHJPPO2024(control_t *control, 
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
	state_array[0] = (state->position.x) / MAX_XY;
	state_array[1] = (state->position.y) / MAX_XY;
	state_array[2] = (state->position.z) / MAX_Z;
	state_array[3] = state->attitudeQuaternion.x;
	state_array[4] = state->attitudeQuaternion.y;
	state_array[5] = state->attitudeQuaternion.z;
	state_array[6] = state->attitudeQuaternion.w;
	state_array[7] = state->velocity.x ;
	state_array[8] = state->velocity.y ;
	state_array[9] = state->velocity.z ;
	state_array[10] = omega_roll;
	state_array[11] = omega_pitch;
	state_array[12] = omega_yaw;
	state_array[13] = control->nn_output[0];
	state_array[14] = control->nn_output[1];
	state_array[15] = control->nn_output[2];
	state_array[16] = control->nn_output[3];

	// run the neural neural network
	uint64_t start = usecTimestamp();
	networkEvaluateHJPPO2024(control, state_array);
	usec_eval = (uint32_t) (usecTimestamp() - start);

	if (setpoint->mode.z == modeDisable) {
		control->normalizedForces[0] = 0.0f;
		control->normalizedForces[1] = 0.0f;
		control->normalizedForces[2] = 0.0f;
		control->normalizedForces[3] = 0.0f;
		control->nn_output[0] = 0.0f;
		control->nn_output[1] = 0.0f;
		control->nn_output[2] = 0.0f;
		control->nn_output[3] = 0.0f;
	}

	last_step_control_t = *control; // update last step_control
}



PARAM_GROUP_START(HJPPO2024)
PARAM_ADD(PARAM_FLOAT, max_thrust, &maxThrustFactor)
PARAM_ADD(PARAM_UINT8, rel_vel, &relVel)
PARAM_ADD(PARAM_UINT8, rel_omega, &relOmega)
PARAM_ADD(PARAM_UINT8, rel_xyz, &relXYZ)
PARAM_ADD(PARAM_UINT16, freq, &freq)
PARAM_GROUP_STOP(HJPPO2024)

LOG_GROUP_START(HJPPO2024)

LOG_ADD(LOG_FLOAT, in0, &state_array[0])
LOG_ADD(LOG_FLOAT, in1, &state_array[1])
LOG_ADD(LOG_FLOAT, in2, &state_array[2])
LOG_ADD(LOG_FLOAT, in3, &state_array[3])
LOG_ADD(LOG_FLOAT, in4, &state_array[4])
LOG_ADD(LOG_FLOAT, in5, &state_array[5])
LOG_ADD(LOG_FLOAT, in6, &state_array[6])
LOG_ADD(LOG_FLOAT, in7, &state_array[7])
LOG_ADD(LOG_FLOAT, in8, &state_array[8])
LOG_ADD(LOG_FLOAT, in9, &state_array[9])
LOG_ADD(LOG_FLOAT, in10, &state_array[10])
LOG_ADD(LOG_FLOAT, in11, &state_array[11])
LOG_ADD(LOG_FLOAT, in12, &state_array[12])

LOG_ADD(LOG_FLOAT, inm1, &state_array[13])
LOG_ADD(LOG_FLOAT, inm2, &state_array[14])
LOG_ADD(LOG_FLOAT, inm3, &state_array[15])
LOG_ADD(LOG_FLOAT, inm4, &state_array[16])
LOG_ADD(LOG_UINT32, usec_eval, &usec_eval)

LOG_GROUP_STOP(HJPPO2024)

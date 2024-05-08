/*
The MIT License (MIT)

Copyright (c) 2024 James A. Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <math.h>
#include <string.h>

#include "debug.h"
#include "math3d.h"
#include "controller_lee.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"

#define FLOAT float
#include "gapsquad.h"


static controllerLee_t g_self = {
	.mass = CF_MASS,
	.arm = ARM_LENGTH,

	// Inertia matrix (diagonal matrix), see
	// System Identification of the Crazyflie 2.0 Nano Quadrocopter
	// BA theses, Julian Foerster, ETHZ
	// https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
	.J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

	.gaps = {
		// main state
		.ierr = {
			0.0f, 0.0f, 0.0f
		},
		.theta = {
			.ki_xy = 0.0,
			.ki_z = 0.0,
			.kp_xy = 7.0,
			.kp_z = 7.0,
			.kv_xy = 4.0,
			.kv_z = 4.0,
			.kr_xy = 19.5,
			.kr_z = 12.5,
			.kw_xy = 3.15,
			.kw_z = 3.15
		},
		.y = { { 0 } },

		// main params
		.cost_param = {
			.p = 1.0f,
			.v = 0.01f,
			.w = 0.01f,
			.thrust = 0.01f,
			.torque = 0.01f
		},
		.eta = 0.0f,
		.damping = 0.9999f,
		.enable = false,
		.optimizer = GAPS_OPT_GRAD,

		// diagnostics
		.yabsmax = 0.0f,
		.sum_cost = 0.0f,

		// AdaDelta
		.grad_accum = { 0 },
		.update_accum = { 0 },
		.ad_decay = 0.9f,  // same as PyTorch.
		.ad_eps = 1e-4    // higher than PyTorch - adapt conservatively.
	}
};

static float const dt = 1.0f/ATTITUDE_RATE;


void controllerLeeReset(controllerLee_t* self)
{
	self->gaps.ierr = vzero();
	// TODO: C
	// Eigen::Map<GapsY>(&self->gaps.y[0][0]).setZero();
	// Eigen::Map<Theta>(&self->gaps.grad_accum[0]).setZero();
	// Eigen::Map<Theta>(&self->gaps.update_accum[0]).setZero();
}

void controllerLeeInit(controllerLee_t* self)
{
	// copy default values (bindings), or NOP (firmware)
	*self = g_self;

	controllerLeeReset(self);
}

bool controllerLeeTest(controllerLee_t* self)
{
	return true;
}

void controllerLee(
	controllerLee_t *self, control_t *control, setpoint_t const *set,
	sensorData_t const *sensors,
	state_t const *state,
	uint32_t const tick)
{

	if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		return;
	}

	static struct State x;
	static struct Target target;
	static struct Action u;

	if (set->mode.yaw == modeAbs) {
		target.y_d = radians(set->attitude.yaw);
	}
	else if (set->mode.quat == modeAbs) {
		struct quat setpoint_quat = mkquat(
			set->attitudeQuaternion.x,
			set->attitudeQuaternion.y,
			set->attitudeQuaternion.z,
			set->attitudeQuaternion.w
		);
		target.y_d = quat2rpy(setpoint_quat).z;
	}
	else {
		DEBUG_PRINT("fail due to unsupported setpoint attitude mode\n");
		goto fail;
	}

	if (set->mode.z == modeDisable && set->thrust < 1000) {
		DEBUG_PRINT("fail due to setpoint modeDisable\n");
		goto fail;
	}
	// we do not handle manual flight with this controller
	if (set->mode.x != modeAbs || set->mode.y != modeAbs || set->mode.z != modeAbs) {
		DEBUG_PRINT("fail due to setpoint not modeAbs\n");
		goto fail;
	}

	target.p_d = mkvec(set->position.x, set->position.y, set->position.z);
	target.v_d = mkvec(set->velocity.x, set->velocity.y, set->velocity.z);
	target.a_d = mkvec(set->acceleration.x, set->acceleration.y, set->acceleration.z);
	// y_d already set above
	target.w_d = mkvec(
		radians(set->attitudeRate.roll),
		radians(set->attitudeRate.pitch),
		radians(set->attitudeRate.yaw)
	);

	x.ierr = self->gaps.ierr;
	x.p = mkvec(state->position.x, state->position.y, state->position.z);
	x.v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
	x.R = quat2rotmat(mkquat(
		state->attitudeQuaternion.x,
		state->attitudeQuaternion.y,
		state->attitudeQuaternion.z,
		state->attitudeQuaternion.w));
	x.w = mkvec(
		radians(sensors->gyro.x),
		radians(sensors->gyro.y),
		radians(sensors->gyro.z)
	);

	bool gaps_ok;
	gaps_ok = gaps_step(&self->gaps, &x, &target, dt, &u);

	if (!gaps_ok) {
		DEBUG_PRINT("fail due to gaps_ok\n");
		goto fail;
	}

	// Reset the accumulated error while on the ground
	if (u.thrust < 1.0f) {
		// Magic number comes from un-dividing original constant by CF mass.
		// Sanity check: It's a lot less than 1g, but not infinitesimal.
		DEBUG_PRINT("fail due to low thrust\n");
		goto fail;
	}

	// output to the rest of the world: convert from normalized to
	// mass/inertia-dependent units.
	control->controlMode = controlModeForceTorque;
	control->thrustSi = self->mass * u.thrust;
	struct vec torque = vscl(1.0f / self->arm, veltmul(self->J, u.torque));
	vstoref(torque, &control->torque[0]);
	return;

fail:
	control->controlMode = controlModeForceTorque;
	control->thrustSi = 0;
	control->torque[0] = 0;
	control->torque[1] = 0;
	control->torque[2] = 0;
	controllerLeeReset(self);
}


#ifdef CRAZYFLIE_FW

#include "param.h"
#include "log.h"

void controllerLeeFirmwareInit(void)
{
	controllerLeeInit(&g_self);
}

bool controllerLeeFirmwareTest(void)
{
	return true;
}

void controllerLeeFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
	controllerLee(&g_self, control, setpoint, sensors, state, tick);
}

PARAM_GROUP_START(gaps6DOF)
	// the GAPS params
	PARAM_ADD(PARAM_UINT8, enable, &g_self.gaps.enable)
	PARAM_ADD(PARAM_UINT8, optimizer, &g_self.gaps.optimizer)
	PARAM_ADD(PARAM_FLOAT, Qp, &g_self.gaps.cost_param.p)
	PARAM_ADD(PARAM_FLOAT, Qv, &g_self.gaps.cost_param.v)
	PARAM_ADD(PARAM_FLOAT, Qw, &g_self.gaps.cost_param.w)
	PARAM_ADD(PARAM_FLOAT, Rthrust, &g_self.gaps.cost_param.thrust)
	PARAM_ADD(PARAM_FLOAT, Rtorque, &g_self.gaps.cost_param.torque)
	PARAM_ADD(PARAM_FLOAT, eta, &g_self.gaps.eta)
	PARAM_ADD(PARAM_FLOAT, damping, &g_self.gaps.damping)
	PARAM_ADD(PARAM_FLOAT, ad_decay, &g_self.gaps.ad_decay)
	PARAM_ADD(PARAM_FLOAT, ad_eps, &g_self.gaps.ad_eps)

	// the controller params (GAPS's theta, for initialization)
	PARAM_ADD(PARAM_FLOAT, ki_xy, &g_self.gaps.theta.ki_xy)
	PARAM_ADD(PARAM_FLOAT, ki_z, &g_self.gaps.theta.ki_z)
	PARAM_ADD(PARAM_FLOAT, kp_xy, &g_self.gaps.theta.kp_xy)
	PARAM_ADD(PARAM_FLOAT, kp_z, &g_self.gaps.theta.kp_z)
	PARAM_ADD(PARAM_FLOAT, kv_xy, &g_self.gaps.theta.kv_xy)
	PARAM_ADD(PARAM_FLOAT, kv_z, &g_self.gaps.theta.kv_z)
	PARAM_ADD(PARAM_FLOAT, kr_xy, &g_self.gaps.theta.kr_xy)
	PARAM_ADD(PARAM_FLOAT, kr_z, &g_self.gaps.theta.kr_z)
	PARAM_ADD(PARAM_FLOAT, kw_xy, &g_self.gaps.theta.kw_xy)
	PARAM_ADD(PARAM_FLOAT, kw_z, &g_self.gaps.theta.kw_z)

	// SystemID params
	PARAM_ADD(PARAM_FLOAT, mass, &g_self.mass)
PARAM_GROUP_STOP(gaps6DOF)


LOG_GROUP_START(gaps6DOF)
	LOG_ADD(LOG_FLOAT, ki_xy, &g_self.gaps.theta.kr_xy)
	LOG_ADD(LOG_FLOAT, ki_z, &g_self.gaps.theta.kr_z)
	LOG_ADD(LOG_FLOAT, kp_xy, &g_self.gaps.theta.kr_xy)
	LOG_ADD(LOG_FLOAT, kp_z, &g_self.gaps.theta.kr_z)
	LOG_ADD(LOG_FLOAT, kv_xy, &g_self.gaps.theta.kr_xy)
	LOG_ADD(LOG_FLOAT, kv_z, &g_self.gaps.theta.kr_z)
	LOG_ADD(LOG_FLOAT, kr_xy, &g_self.gaps.theta.kr_xy)
	LOG_ADD(LOG_FLOAT, kr_z, &g_self.gaps.theta.kr_z)
	LOG_ADD(LOG_FLOAT, kw_xy, &g_self.gaps.theta.kr_xy)
	LOG_ADD(LOG_FLOAT, kw_z, &g_self.gaps.theta.kr_z)
	LOG_ADD(LOG_FLOAT, yabsmax, &g_self.gaps.yabsmax)

	// LOG_ADD(LOG_FLOAT, thrustSi, &g_self.thrustSi)
	// LOG_ADD(LOG_FLOAT, torquex, &g_self.u.x)
	// LOG_ADD(LOG_FLOAT, torquey, &g_self.u.y)
	// LOG_ADD(LOG_FLOAT, torquez, &g_self.u.z)

	// current angles
	// LOG_ADD(LOG_FLOAT, rpyx, &g_self.rpy.x)
	// LOG_ADD(LOG_FLOAT, rpyy, &g_self.rpy.y)
	// LOG_ADD(LOG_FLOAT, rpyz, &g_self.rpy.z)

	// desired angles
	// LOG_ADD(LOG_FLOAT, rpydx, &g_self.rpy_des.x)
	// LOG_ADD(LOG_FLOAT, rpydy, &g_self.rpy_des.y)
	// LOG_ADD(LOG_FLOAT, rpydz, &g_self.rpy_des.z)

	// errors
	// LOG_ADD(LOG_FLOAT, error_posx, &g_self.p_error.x)
	// LOG_ADD(LOG_FLOAT, error_posy, &g_self.p_error.y)
	// LOG_ADD(LOG_FLOAT, error_posz, &g_self.p_error.z)

	// LOG_ADD(LOG_FLOAT, error_velx, &g_self.v_error.x)
	// LOG_ADD(LOG_FLOAT, error_vely, &g_self.v_error.y)
	// LOG_ADD(LOG_FLOAT, error_velz, &g_self.v_error.z)

	// omega
	// LOG_ADD(LOG_FLOAT, omegax, &g_self.omega.x)
	// LOG_ADD(LOG_FLOAT, omegay, &g_self.omega.y)
	// LOG_ADD(LOG_FLOAT, omegaz, &g_self.omega.z)

	// omega_r
	// LOG_ADD(LOG_FLOAT, omegarx, &g_self.omega_r.x)
	// LOG_ADD(LOG_FLOAT, omegary, &g_self.omega_r.y)
	// LOG_ADD(LOG_FLOAT, omegarz, &g_self.omega_r.z)

	// LOG_ADD(LOG_UINT32, ticks, &ticks)
LOG_GROUP_STOP(gaps6DOF)

#endif // CRAZYFLIE_FW defined

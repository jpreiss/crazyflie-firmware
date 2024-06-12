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

// fixed-point x (1 << 11), for logging
static struct {
	int16_t ki_xy;
	int16_t ki_z;
	int16_t kp_xy;
	int16_t kp_z;
	int16_t kv_xy;
	int16_t kv_z;
	int16_t kr_xy;
	int16_t kr_z;
	int16_t kw_xy;
	int16_t kw_z;
} g_log;

static controllerLee_t g_self = {
	.mass = CF_MASS,

	// Inertia matrix (diagonal matrix), see
	// System Identification of the Crazyflie 2.0 Nano Quadrocopter
	// BA theses, Julian Foerster, ETHZ
	// https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
	.J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

	.prev_w_err = {0, 0, 0},
	.kdw_xy = 2.36f,

	.gaps = {
		// main state
		.ierr = {
			0.0f, 0.0f, 0.0f
		},
		.theta = {
			.ki_xy = 0.444686,
			.ki_z  = 0.444686,
			.kp_xy = 2.525729,
			.kp_z  = 3.663562,
			.kv_xy = 1.832581,
			.kv_z  = 2.525729,
			.kr_xy = 7.414573,
			.kr_z  = 5.683580,
			.kw_xy = 5.470168,
			.kw_z  = 3.391147,
		},
		.y = { { 0 } },

		// main params
		.cost_param = {
			.p = 1.0f,
			// default to 0 regularization because it's easy to add too much!
			.v = 0,
			.w = 0,
			.thrust = 0,
			.torque = 0,
			.reg_L2 = 0,
		},
		.eta = 0.0f,
		.damping = 0.9999f,
		.enable = false,
		.optimizer = GAPS_OPT_GRAD,
		.gradmode = GAPS_GRAD_LOGPARAM,

		.single_point = {
			.perturbation = { 0 },
			.radius = 1e-1,
			.ep_len = ATTITUDE_RATE * 1, // TODO: tune episode length
			.cost_accum = 0,
			.ep_step = ATTITUDE_RATE * 1,
		},

		.actor_critic = {
			.init = false,
			// other states will be lazy-initialized by actor-critic algo.
			.critic_rate = 0.0f,
			.gamma = 0.9999f,
		},

		// diagnostics
		.yabsmax = 0.0f,
		.max_row = -1,
		.max_col = -1,
		.sum_cost = 0.0f,
		.debug = {
			.z_axis_desired = {0, 0, 0},
			.eR = {0, 0, 0},
			.ew = {0, 0, 0}
		},

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

	float const FIXPT = 1 << 11;
	g_log.ki_xy = self->gaps.theta.ki_xy * FIXPT;
	g_log.ki_z  = self->gaps.theta.ki_z  * FIXPT;
	g_log.kp_xy = self->gaps.theta.kp_xy * FIXPT;
	g_log.kp_z  = self->gaps.theta.kp_z  * FIXPT;
	g_log.kv_xy = self->gaps.theta.kv_xy * FIXPT;
	g_log.kv_z  = self->gaps.theta.kv_z  * FIXPT;
	g_log.kr_xy = self->gaps.theta.kr_xy * FIXPT;
	g_log.kr_z  = self->gaps.theta.kr_z  * FIXPT;
	g_log.kw_xy = self->gaps.theta.kw_xy * FIXPT;
	g_log.kw_z  = self->gaps.theta.kw_z  * FIXPT;

	static struct State x;
	static struct Target target;
	static struct Action u;

	if (set->mode.yaw != modeAbs && set->mode.quat != modeAbs) {
		// This happens routinely, i.e. after startup on ground.
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
	x.logR = qlog(mkquat(
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

	// We keep this out of the GAPS because its dynamics cannot be modeled as
	// anything other than a disturbance.
	struct vec w_err = vsub(x.w, target.w_d);
	struct vec dw_err = vscl(1.0f / dt, vsub(w_err, self->prev_w_err));
	dw_err.z = 0.0f;
	u.torque = vsub(u.torque, vscl(self->kdw_xy, dw_err));
	self->prev_w_err = w_err;

	// output to the rest of the world: convert from normalized to
	// mass/inertia-dependent units.
	control->controlMode = controlModeForceTorque;
	control->thrustSi = self->mass * u.thrust;
	struct vec torque = veltmul(self->J, u.torque);
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
	PARAM_ADD(PARAM_UINT8, gradmode, &g_self.gaps.gradmode)
	PARAM_ADD(PARAM_FLOAT, Qp, &g_self.gaps.cost_param.p)
	PARAM_ADD(PARAM_FLOAT, Qv, &g_self.gaps.cost_param.v)
	PARAM_ADD(PARAM_FLOAT, Qw, &g_self.gaps.cost_param.w)
	PARAM_ADD(PARAM_FLOAT, Rthrust, &g_self.gaps.cost_param.thrust)
	PARAM_ADD(PARAM_FLOAT, Rtorque, &g_self.gaps.cost_param.torque)
	PARAM_ADD(PARAM_FLOAT, reg_L2, &g_self.gaps.cost_param.reg_L2)
	PARAM_ADD(PARAM_FLOAT, eta, &g_self.gaps.eta)
	PARAM_ADD(PARAM_FLOAT, damping, &g_self.gaps.damping)
	PARAM_ADD(PARAM_FLOAT, ad_decay, &g_self.gaps.ad_decay)
	PARAM_ADD(PARAM_FLOAT, ad_eps, &g_self.gaps.ad_eps)
	// single-point PG stuff
	PARAM_ADD(PARAM_UINT32, sp_ep_len, &g_self.gaps.single_point.ep_len)
	PARAM_ADD(PARAM_FLOAT, sp_radius, &g_self.gaps.single_point.radius)
	// actor-critic stuff
	PARAM_ADD(PARAM_FLOAT, ac_gamma, &g_self.gaps.actor_critic.gamma)
	PARAM_ADD(PARAM_FLOAT, ac_rate, &g_self.gaps.actor_critic.critic_rate)

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
	PARAM_ADD(PARAM_FLOAT, kdw_xy, &g_self.kdw_xy)

	// SystemID params
	PARAM_ADD(PARAM_FLOAT, mass, &g_self.mass)
PARAM_GROUP_STOP(gaps6DOF)


LOG_GROUP_START(gaps6DOF)
	LOG_ADD(LOG_INT16, ki_xy, &g_log.ki_xy)
	LOG_ADD(LOG_INT16, ki_z,  &g_log.ki_z)
	LOG_ADD(LOG_INT16, kp_xy, &g_log.kp_xy)
	LOG_ADD(LOG_INT16, kp_z,  &g_log.kp_z)
	LOG_ADD(LOG_INT16, kv_xy, &g_log.kv_xy)
	LOG_ADD(LOG_INT16, kv_z,  &g_log.kv_z)
	LOG_ADD(LOG_INT16, kr_xy, &g_log.kr_xy)
	LOG_ADD(LOG_INT16, kr_z,  &g_log.kr_z)
	LOG_ADD(LOG_INT16, kw_xy, &g_log.kw_xy)
	LOG_ADD(LOG_INT16, kw_z,  &g_log.kw_z)
	LOG_ADD(LOG_FLOAT, yabsmax, &g_self.gaps.yabsmax)
	LOG_ADD(LOG_FLOAT, sum_cost, &g_self.gaps.sum_cost)
	LOG_ADD(LOG_INT8, max_row, &g_self.gaps.max_row)
	LOG_ADD(LOG_INT8, max_col, &g_self.gaps.max_col)
	LOG_ADD(LOG_FLOAT, ix, &g_self.gaps.ierr.x)
	LOG_ADD(LOG_FLOAT, iy, &g_self.gaps.ierr.y)
	LOG_ADD(LOG_FLOAT, iz, &g_self.gaps.ierr.z)
	LOG_ADD(LOG_FLOAT, squash_x, &g_self.gaps.debug.dw_squash.x)
	LOG_ADD(LOG_FLOAT, squash_y, &g_self.gaps.debug.dw_squash.y)
	LOG_ADD(LOG_FLOAT, squash_z, &g_self.gaps.debug.dw_squash.z)
	LOG_ADD(LOG_FLOAT, er_x, &g_self.gaps.debug.eR.x)
	LOG_ADD(LOG_FLOAT, er_y, &g_self.gaps.debug.eR.y)
	LOG_ADD(LOG_FLOAT, er_z, &g_self.gaps.debug.eR.z)

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

#include "solve6x6.h"
#include "math3d.h"
#include "tilthex_control.h"

#include "param.h"

// control gains.
// these are very low gain, about 10x is better, but default should be gentle...

// position
static float kP_p = 10;
static float kP_d = 5;
// rotational
static float kR_p = 10;
static float kR_d = 5;

// system identification parameters.
static float mass = 0.7; // kg
static float prop_rpm_max = 20000;
static float inertia1 = 0.01;
static float inertia2 = 0.01;
static float inertia3 = 0.02;
static float thrust_constant = 0.5e-5;
static float drag_constant = 2.5e-7;

void compute_f(struct tilthex_state s, struct tilthex_state des, float f[6])
{
	struct vec err_pos = vsub(s.pos, des.pos);
	struct vec err_vel = vsub(s.vel, des.vel);
	struct vec v_p = vadd3(
		des.acc,
		vscl(-kP_p, err_pos),
		vscl(-kP_d, err_vel)
	);

	struct vec e_R = vscl(0.5, mvee(
		msub(
			mmult(mtranspose(des.R), s.R),
			mmult(mtranspose(s.R), des.R))
	));
	struct vec e_omega = vsub(s.omega,
		mvmult(mmult(mtranspose(s.R), des.R), des.omega));
	
	struct vec v_R = vadd(vscl(-kR_p, e_R), vscl(-kR_d, e_omega));

	vstoref(v_p, &f[0]);
	vstoref(v_R, &f[3]);

	f[2] = f[2] + 9.81f;
}

void tilthex_control(struct tilthex_state s, struct tilthex_state des, float x[6])
{
	float J[6][6] = {{0}};
	#include "F2.cprocessed.c"

	// construct the Jacobian.
	struct mat33 F1_left = mzero();
	#include "F1_left.cprocessed.c"

	struct mat33 F1_right = mzero();
	#include "F1_right.cprocessed.c"

	struct mat33 J11 = mmult(s.R, F1_left);
	struct mat33 J12 = mmult(s.R, F1_right);
	set_block33_rowmaj(&J[0][0], 6, &J11);
	set_block33_rowmaj(&J[0][3], 6, &J12);

	// Jacobian done.

	// construct rhs and solve.
	float fmv[6];
	compute_f(s, des, fmv);
	solve6x6(J, fmv, x);

	float const omega_max = ((float)(2*M_PI/60)) * prop_rpm_max;
	float const omega2_max = omega_max * omega_max;
	for (int i = 0; i < 6; ++i) {
		x[i] = fmax(fmin(x[i], omega2_max), 0.0f);
	}
}

PARAM_GROUP_START(tilthex_pid)
PARAM_ADD(PARAM_FLOAT, pos_kp, &kP_p)
PARAM_ADD(PARAM_FLOAT, pos_kd, &kP_d)
PARAM_ADD(PARAM_FLOAT, att_kp, &kR_p)
PARAM_ADD(PARAM_FLOAT, att_kd, &kR_d)
PARAM_GROUP_STOP(tilthex_pid)

PARAM_GROUP_START(tilthex_dynamics)
PARAM_ADD(PARAM_FLOAT, mass, &mass)
PARAM_ADD(PARAM_FLOAT, rpm_max, &prop_rpm_max)
PARAM_ADD(PARAM_FLOAT, inertia1, &inertia1)
PARAM_ADD(PARAM_FLOAT, inertia2, &inertia2)
PARAM_ADD(PARAM_FLOAT, inertia3, &inertia3)
PARAM_ADD(PARAM_FLOAT, k_thrust, &thrust_constant)
PARAM_ADD(PARAM_FLOAT, k_drag, &drag_constant)
PARAM_GROUP_STOP(tilthex_dynamics)

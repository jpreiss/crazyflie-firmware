//#include "solve6x6.h"
#include "cvxgen/solver.h"
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
static float prop_rpm_max = 22000; // approx.
// TODO: real inertia parameters
static float inertia1 = 0.01f;
static float inertia2 = 0.01f;
static float inertia3 = 0.02f;
// measured using tachometer + scale
static float thrust_constant = 1.6e-6f;
// the drag constant is harder to measure than thrust.
// this value is determined based on the rough average
//      drag_constant = 0.05 * thrust_constant
// from system identification papers in the literature.
// however, reported values ranged from 0.005 to 0.1,
// so this is quite approximate and might need tuning.
static float drag_constant = 8.0e-8f;

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

struct Vars_t vars;
struct Params_t params;
struct Workspace_t work;
struct Settings_t settings;

void tilthex_control(struct tilthex_state s, struct tilthex_state des, float x[6])
{
	// construct the Jacobian using generated code from Matlab
	float J[6][6] = {{0}};
	struct mat33 F1_left = mzero();
	struct mat33 F1_right = mzero();
	#include "jacobian_fill_generated.c"

	struct mat33 J11 = mmult(s.R, F1_left);
	struct mat33 J12 = mmult(s.R, F1_right);
	set_block33_rowmaj(&J[0][0], 6, &J11);
	set_block33_rowmaj(&J[0][3], 6, &J12);

	// construct rhs and solve.
	float fmv[6];
	compute_f(s, des, fmv);

	float const OMEGA_MAX = 4.3865e6; // 20,000 RPM

	// solve box constrained linear least squares thrust mixing with CVXGEN
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			params.A[i+6*j] = OMEGA_MAX * J[i][j];
		}
		params.b[i] = fmv[i];
	}

	// TODO move some setup stuff to init
	// TODO warm start ???
	set_defaults();
	settings.verbose = 0;
	//settings.max_iters = 10;
	settings.eps = 0.1;
	settings.resid_tol = 0.05;
	setup_indexing();
	solve();

	// cvxgen thrust mixing set to loose tolerance, 
	// so may not produce exactly valid omegas. clipping still needed.
	for (int i = 0; i < 6; ++i) {
		float omega = OMEGA_MAX * vars.x[i];
		x[i] = fmax(fmin(omega, OMEGA_MAX), 0.0f);
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

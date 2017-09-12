#include "cvxgen/solver.h"
#include "math3d.h"
#include "solve6x6.h"
#include "tilthex_control.h"

#include "param.h"

// control gains.
// these are very low gain, about 10x is better, but default should be gentle...

// position
static float kP_p = 10;
static float kP_i = 5;
static float kP_d = 5;
static float satP_i = 0.1; // saturation - meters * seconds

// rotational
static float kR_p = 10;
static float kR_d = 5;

// system identification parameters.
static float mass = 1.096; // kg
static float prop_rpm_max = 22000; // approx.
// inerita matrix computed from CAD model
static float inertia1 = 0.0119f;
static float inertia2 = 0.0121f;
static float inertia3 = 0.0234f;
// measured using tachometer + scale
static float thrust_constant = 1.6e-6f;
// the drag constant is harder to measure than thrust.
// this value is determined based on the rough average
//      drag_constant = 0.05 * thrust_constant
// from system identification papers in the literature.
// however, reported values ranged from 0.005 to 0.1,
// so this is quite approximate and might need tuning.
static float drag_constant = 8.0e-8f;

// it should be a bool but something is weird w/ 8-bit log/param!
static uint16_t use_cvxgen = 0;

static struct vec update_integral(struct vec integral, struct vec err_pos, float dt)
{
	return vclamp(vadd(integral, vscl(dt, err_pos)), -satP_i, satP_i);
}

void compute_f(struct tilthex_state *s, struct tilthex_state des, float dt, float f[6])
{
	struct vec err_pos = vsub(s->pos, des.pos);
	struct vec err_vel = vsub(s->vel, des.vel);
	s->pos_err_integral = update_integral(s->pos_err_integral, err_pos, dt);
	struct vec v_p = vadd4(
		des.acc,
		vscl(-kP_p, err_pos),
		vscl(-kP_d, err_vel),
		vscl(-kP_i, s->pos_err_integral)
	);

	struct vec e_R = vscl(0.5, mvee(
		msub(
			mmult(mtranspose(des.R), s->R),
			mmult(mtranspose(s->R), des.R))
	));
	struct vec e_omega = vsub(s->omega,
		mvmult(mmult(mtranspose(s->R), des.R), des.omega));
	
	struct vec v_R = vadd(vscl(-kR_p, e_R), vscl(-kR_d, e_omega));

	struct vec inertia_diag = mkvec(inertia1, inertia2, inertia3);
	struct vec inertia_inv_diag = veltrecip(inertia_diag);
	struct vec dw_inertia = vneg(veltmult(
		inertia_inv_diag,
		vcross(s->omega, veltmult(inertia_diag, s->omega))));

	v_R = vsub(v_R, dw_inertia);

	vstoref(v_p, &f[0]);
	vstoref(v_R, &f[3]);

	f[2] = f[2] + 9.81f;
}

struct Vars_t vars;
struct Params_t params;
struct Workspace_t work;
struct Settings_t settings;


void tilthex_control(struct tilthex_state *s, struct tilthex_state const des, float dt, float x[6])
{
	// construct the Jacobian using generated code from Matlab
	float J[6][6] = {{0}};
	struct mat33 F1_left = mzero();
	struct mat33 F1_right = mzero();
	#include "jacobian_fill_generated.c"

	struct mat33 J11 = mmult(s->R, F1_left);
	struct mat33 J12 = mmult(s->R, F1_right);
	set_block33_rowmaj(&J[0][0], 6, &J11);
	set_block33_rowmaj(&J[0][3], 6, &J12);

	// construct rhs and solve.
	float fmv[6];
	compute_f(s, des, dt, fmv);

	// scale the Jacobian for better conditioning.
	float const omega2_max = fsqr((2.0f * M_PI_F / 60.0f) * prop_rpm_max);
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			J[i][j] = omega2_max * J[i][j];
		}
	}

	if (use_cvxgen) {
		// solve box constrained linear least squares thrust mixing with CVXGEN
		for (int i = 0; i < 6; ++i) {
			for (int j = 0; j < 6; ++j) {
				params.A[i+6*j] = J[i][j];
			}
			params.b[i] = fmv[i];
		}

		// TODO move some setup stuff to init
		// TODO warm start ???
		set_defaults();
		settings.verbose = 0;
		settings.max_iters = 10;
		settings.eps = 0.1;
		settings.resid_tol = 0.05;
		setup_indexing();
		solve();

		// cvxgen thrust mixing set to loose tolerance, 
		// so may not produce exactly valid omegas. clipping still needed,
		// see below.
		for (int i = 0; i < 6; ++i) {
			x[i] = vars.x[i];
		}
	}
	else {
		solve6x6(J, fmv, x);
	}

	// clipping and scaling back up to omega^2 units.
	for (int i = 0; i < 6; ++i) {
		x[i] = omega2_max * fmax(fmin(x[i], 1.0f), 0.0f);
	}
}

PARAM_GROUP_START(tilthex_pid)
PARAM_ADD(PARAM_FLOAT, pos_kp, &kP_p)
PARAM_ADD(PARAM_FLOAT, pos_ki, &kP_i)
PARAM_ADD(PARAM_FLOAT, pos_kd, &kP_d)
PARAM_ADD(PARAM_FLOAT, pos_satp, &satP_i)
PARAM_ADD(PARAM_FLOAT, att_kp, &kR_p)
PARAM_ADD(PARAM_FLOAT, att_kd, &kR_d)
PARAM_ADD(PARAM_FLOAT, att_kd, &kR_d)
PARAM_ADD(PARAM_UINT16, use_cvxgen, &use_cvxgen)
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

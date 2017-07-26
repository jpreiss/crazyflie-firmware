#include "solve6x6.h"
#include "math3d.h"
#include "tilthex_control.h"

void compute_f(struct tilthex_state s, struct tilthex_state des, float f[6])
{
	float kP_p = 100;
	float kP_d = 50;

	float kR_p = 100;
	float kR_d = 50;

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

void control(struct tilthex_state s, struct tilthex_state des, float x[6])
{
	// construct the Jacobian.
	struct mat33 F1_left = {
		.m = {
			{                 0,   0.309294787065871,  -0.309294787065871 },
			{ 0.357142857142857,  -0.178571428571429,  -0.178571428571428 },
			{ 0.618589574131742,   0.618589574131742,   0.618589574131742 }}
	};
	F1_left = mscale(1.0e-5, F1_left);

	struct mat33 F1_right = {
		.m = {
			{ 0.000000000000000,  0.309294787065871, -0.309294787065871 },
			{ 0.357142857142857, -0.178571428571429, -0.178571428571429 },
			{ 0.618589574131742,  0.618589574131742,  0.618589574131742 }}
	};
	F1_right = mscale(1.0e-5, F1_right);

	float J[6][6] = {
		{0, },
		{0, },
		{0, },
		{0,  0.858253175473055,  0.858253175473055,  0.000000000000000, -0.858253175473055, -0.858253175473055 },
		{-0.991025403784439, -0.495512701892220,  0.495512701892219,  0.991025403784439,  0.495512701892220, -0.495512701892219 },
		{0.141746824526945, -0.141746824526945,  0.141746824526945, -0.141746824526945,  0.141746824526945, -0.141746824526945 }
	};

	for (int r = 3; r < 6; ++r) {
		for (int c = 0; c < 6; ++c) {
			J[r][c] = 1.0e-4f * J[r][c];
		}
	}

	struct mat33 J11 = mmult(s.R, F1_left);
	struct mat33 J12 = mmult(s.R, F1_right);

	set_block33_rowmaj(&J[0][0], 6, &J11);
	set_block33_rowmaj(&J[0][3], 6, &J12);

	// Jacobian done.

	// construct rhs and solve.
	float fmv[6];
	compute_f(s, des, fmv);
	solve6x6(J, fmv, x);

	float const OMEGA_MAX = 1.096622711232151e6;
	for (int i = 0; i < 6; ++i) {
		x[i] = fmax(fmin(x[i], OMEGA_MAX), 0.0f);
	}
}

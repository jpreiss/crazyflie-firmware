#pragma once

/*
C++ Style Note
--------------

Distutils (used for the firmware python bindings, essential for simulation
testing) will happily compile an extension containing both C and C++ sources,
but it wants to always use the C compiler for both. Clang will switch to C++
mode based on file extension, but then uses a weird standard that allows some
C++11 features (such as `auto`) but disallows others (such as brace
initialization). Unfortunately Clang also rejects passing a `--std=c++*`
argument - you must invoke it as clang++ to allow that. I spent several hours
trying to figure out a way to make distutils invoke a C++ compiler for C++
sources, but eventually gave up and adopted its permitted features.

The main feature sacrificed was `std::tuple` return types. We use output
reference parameters instead.
*/

#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/KroneckerProduct>

#include "gapsquad.h"
#include "codegen/cost.h"
#include "codegen/ctrl.h"
#include "codegen/dynamics.h"

#ifdef CRAZYFLIE_FW
	extern "C" {
		#include "debug.h"
	}
#else
	int const EXC_MSG_BUFSZ = 512;
	static char exc_msg_buf[EXC_MSG_BUFSZ] = { 0 };
	#define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif

using Theta = Eigen::Array<FLOAT, TDIM, 1>;
using GapsY = Eigen::Matrix<FLOAT, XDIM, TDIM, Eigen::RowMajor>;
using Jxx = Eigen::Matrix<FLOAT, XDIM, XDIM, Eigen::RowMajor>;
using Jxu = Eigen::Matrix<FLOAT, XDIM, UDIM, Eigen::RowMajor>;
using Jut = Eigen::Matrix<FLOAT, UDIM, TDIM, Eigen::RowMajor>;
using Jux = Eigen::Matrix<FLOAT, UDIM, XDIM, Eigen::RowMajor>;
using Gcx = Eigen::Matrix<FLOAT, 1, XDIM>;
using Gcu = Eigen::Matrix<FLOAT, 1, UDIM>;

FLOAT const GRAV = 9.81;
using Mat39 = Eigen::Matrix<FLOAT, 3, 9>;
using Mat93 = Eigen::Matrix<FLOAT, 9, 3>;
using Mat99 = Eigen::Matrix<FLOAT, 9, 9>;
using VecT = Eigen::Matrix<FLOAT, 1, 3>;
using Diag = Eigen::DiagonalMatrix<FLOAT, 3>;
using Arr3 = Eigen::Array<FLOAT, 3, 1>;

void colsplit(Mat const &m, Vec &x, Vec &y, Vec &z)
{
	x = m.col(0);
	y = m.col(1);
	z = m.col(2);
}

Mat fromcols(Vec const &a, Vec const &b, Vec const &c)
{
	Mat m;
	m.col(0) = a;
	m.col(1) = b;
	m.col(2) = c;
	return m;
}

/*
Returns error on Lie algebra, plus Jacobians (3 x 9).

Note this error starts *decreasing* as the angle exceeds 90 degrees, so it is
nonsensical. Also it has a negative second derivative so it really only makes
sense for small angles like 45 degrees or less (see [1] for details).

However, we use it here because its Jacobian is so simple.

[1] Globally-Attractive Logarithmic Geometric Control of a Quadrotor for
Aggressive Trajectory Tracking. Jacob Johnson and Randal Beard.
https://arxiv.org/abs/2109.07025
*/
Vec SO3error(Mat const &R, Mat const &Rd, Mat39 &JR, Mat39 &JRd)
{
	Mat errmat = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
	Vec err(errmat(2, 1), errmat(0, 2), errmat(1, 0));
	Vec Rx, Ry, Rz;
	colsplit(R, Rx, Ry, Rz);
	Vec Rdx, Rdy, Rdz;
	colsplit(Rd, Rdx, Rdy, Rdz);
	VecT Z = VecT::Zero();
	JR = 0.5 * (Mat39() <<
		               Z,  Rdz.transpose(), -Rdy.transpose(),
		-Rdz.transpose(),                Z,  Rdx.transpose(),
		 Rdy.transpose(), -Rdx.transpose(),               Z).finished();
	JRd = 0.5 * (Mat39() <<
		              Z, -Rz.transpose(),  Ry.transpose(),
		 Rz.transpose(),               Z, -Rx.transpose(),
		-Ry.transpose(),  Rx.transpose(),              Z).finished();
	return err;
}

Mat hat(Vec const &w)
{
	FLOAT x = w[0], y = w[1], z = w[2];
	Mat m = (Mat() <<
		 0, -z,  y,
		 z,  0, -x,
		-y,  x,  0).finished();
	return m;
}

static Mat93 const Dhat_w = (Mat93() <<
	 0,  0,  0,
	 0,  0,  1,
	 0, -1,  0,
	 0,  0, -1,
	 0,  0,  0,
	 1,  0,  0,
	 0,  1,  0,
	-1,  0,  0,
	 0,  0,  0).finished();

Vec normalize(Vec const &v, Mat &J)
{
	FLOAT vn = (FLOAT)1.0 / v.norm();
	FLOAT vn3 = vn * vn * vn;
	J = vn * Mat::Identity() - vn3 * v * v.transpose();
	return vn * v;
}

Vec cross(Vec const &a, Vec const &b, Mat &Ja, Mat &Jb)
{
	FLOAT ax = a[0], ay = a[1], az = a[2];
	FLOAT bx = b[0], by = b[1], bz = b[2];
	Vec x(
		ay * bz - az * by,
		az * bx - ax * bz,
		ax * by - ay * bx
	);
	Ja = -hat(b);
	Jb = hat(a);
	return x;
}


void dynamics(
	State const &x, Target const &t, Action const &u, FLOAT const dt, // inputs
	State &x_t, Jxx &Dx_x, Jxu &Dx_u // outputs
	)
{
	Eigen::Matrix<FLOAT, XDIM, 1> xt;
	Eigen::Matrix<FLOAT, XDIM, XDIM + UDIM> D;
	sym::Dynamics<FLOAT>(
		x.ierr, x.p, x.v, x.R, x.w,
		t.p_d,
		u.thrust, u.torque,
		dt,
		&xt, &D
	);
	using Rmap = Eigen::Map<Eigen::Matrix<FLOAT, 3, 3, Eigen::ColMajor> >;
	x_t.ierr = xt.head<3>();
	x_t.p = xt.segment<3>(3);
	x_t.v = xt.segment<3>(6),
	x_t.R = Rmap(&xt[9]);
	x_t.w = xt.tail<3>();
	Dx_x = D.block<XDIM, XDIM>(0, 0);
	Dx_u = D.block<XDIM, UDIM>(0, XDIM);
}

template <typename S, typename T>
bool allclose(S &&s, T &&t, FLOAT atol=1e-8, FLOAT rtol=1e-5)
{
	auto a = (s - t).array().abs();
	auto allowed = rtol * t.array().abs() + atol;
	return (a < allowed).all();
}

void cost(
	State const &x, Target const &t, Action const &u, CostParam const &Q,  // inputs
	FLOAT &c, Gcx &Dc_x, Gcu &Dc_u  // outputs
	)
{
	Eigen::Matrix<FLOAT, 1, 3 + 3 + 3 + 1 + 3> D;
	//                      p   v   w   th  tq
	sym::Cost<FLOAT>(
		x.p, x.v, x.w,
		t.p_d, t.v_d, t.w_d,
		u.thrust, u.torque,
		Q.p, Q.v, Q.w, Q.thrust, Q.torque,
		&c, &D
	);
	Dc_x.setZero();
	Dc_x.segment<6>(3) = D.head<6>();  // p, v
	Dc_x.tail<3>() = D.segment<3>(6);  // w
	Dc_u[0] = D[9];                    // thrust
	Dc_u.tail<3>() = D.tail<3>();      // torque
}


void ctrl(
	State const &x, Target const &t, Param const &th, // inputs
	Action &u, Jux &Du_x, Jut &Du_th, // main outputs
	Debug &debug,
	FLOAT dt // params
	)
{
	static_assert(TDIM == 6 + 4);
	Eigen::Matrix<FLOAT, 6, 1> th_pos;
	th_pos << th.ki_xy, th.ki_z, th.kp_xy, th.kp_z, th.kv_xy, th.kv_z;
	Eigen::Matrix<FLOAT, 4, 1> th_rot;
	th_rot << th.kr_xy, th.kr_z, th.kw_xy, th.kw_z;

	Eigen::Matrix<FLOAT, UDIM, 1> ua;
	Eigen::Matrix<FLOAT, UDIM, XDIM + TDIM> D;
	sym::Ctrl<FLOAT>(
		x.ierr, x.p, x.v, x.R, x.w,
		t.p_d, t.v_d, t.a_d, t.y_d, t.w_d,
		th_pos, th_rot,
		dt,
		&ua, &D
	);
	u.thrust = ua[0];
	u.torque = ua.tail<3>();
	Du_x = D.block<UDIM, XDIM>(0, 0);
	Du_th = D.block<UDIM, TDIM>(0, XDIM);
}

// spiritually these are function-static to gaps_step(), but we make them fully
// global to avoid g++ emitting the __cxa_guard* calls, which are not available
// in CF firmware and apply only to function-static variables.
static Jux Du_x;
static Jut Du_t;
static State xnext;
static Jxx Dx_x;
static Jxu Dx_u;
static Gcx Dc_x;
static Gcu Dc_u;

template <typename T>
T clampsym(T const &x, T const &absmax)
{
	if (x < -absmax) return -absmax;
	if (x > absmax) return absmax;
	return x;
}

extern "C" bool gaps_step(
	struct GAPS *gaps,
	struct State const *x,
	struct Target const *t,
	FLOAT const dt,
	struct Action *u_out)
{
	ctrl(*x, *t, gaps->theta, *u_out, Du_x, Du_t, gaps->debug, dt);

	// integrate the ierr right away in case we exit due to being disabled.
	gaps->ierr += dt * (x->p - t->p_d);
	gaps->ierr[0] = clampsym(gaps->ierr[0], (FLOAT)2.0);
	gaps->ierr[1] = clampsym(gaps->ierr[1], (FLOAT)2.0);
	gaps->ierr[2] = clampsym(gaps->ierr[2], (FLOAT)0.4);

	if (!gaps->enable) {
		return true;
	}

	dynamics(*x, *t, *u_out, dt, xnext, Dx_x, Dx_u);

	FLOAT stage_cost;
	cost(*x, *t, *u_out, gaps->cost_param, stage_cost, Dc_x, Dc_u);
	gaps->sum_cost += dt * stage_cost;

	using MapTheta = Eigen::Map<Theta>;

	// 1) compute the gradient
	Eigen::Map<GapsY> y(gaps->y[0]);
	Theta grad = (Dc_x * y + Dc_u * Du_t).array();
	MapTheta theta((FLOAT *)&gaps->theta);
	// regularization
	grad += gaps->cost_param.reg_L2 * theta;

	// 2) run the optimizer
	gaps_optimizer const opt = (gaps_optimizer)gaps->optimizer;
	if (opt == GAPS_OPT_GRAD) {
		theta -= gaps->eta * grad;
	}
	else if (opt == GAPS_OPT_ADADELTA) {
		// AdaDelta dynamics - notation follows paper
		FLOAT const decay = gaps->ad_decay;
		MapTheta ad_grads(gaps->grad_accum);
		MapTheta ad_updates(gaps->update_accum);
		ad_grads = decay * ad_grads + (1.0f - decay) * grad.square();
		Theta numerator = (gaps->ad_eps + ad_updates.array()).sqrt();
		Theta denominator = (gaps->ad_eps + ad_grads.array()).sqrt();
		Theta scale = numerator / denominator;
		Theta update = -scale * grad.array();
		ad_updates = decay * ad_updates + (1.0f - decay) * update.square();
		theta += gaps->eta * update;
	}
	else {
		return false;
	}

	// GAPS 3) update the state
	// projection
	theta = theta.max(0.0f);
	// dynamic programming (with slight damping)
	auto product = Dx_x + Dx_u * Du_x;
	// std::cout << "product eigvals:" << product.eigenvalues() << "\n";
	y = gaps->damping * product * y + Dx_u * Du_t;
	// enforce that the R component of y is within the tangent space of R
	auto y_R = y.block<9, TDIM>(9, 0);

	// project the R components of y onto the tangent space of current R
	for (int i = 0; i < TDIM; ++i) {
		Eigen::Matrix<FLOAT, 9, 1> y_R_i = y_R.col(i);

		// y_R_i_M is analogous to R^T since we store concatenated cols but Mat is row-major
		Eigen::Map<Mat> y_R_i_M(y_R_i.data());

		// move from tangent space of R to tangent space of identity
		Mat in_tangent = xnext.R.transpose() * y_R_i_M.transpose();

		// enforce to be in so(3) Lie algebra
		Mat skew_part = 0.5 * (in_tangent - in_tangent.transpose());

		if (!allclose(skew_part, -skew_part.transpose())) {
			DEBUG_PRINT("skew projection failed.\n");
		}

		// go back to tangent space of R
		y_R_i_M = (xnext.R * skew_part).transpose();

		// FLOAT relative = (y_R_i - y_R.col(i)).norm() / y_R.col(i).norm();
		// DEBUG_PRINT("skew proj relative %f\n", (double)relative);

		// remember y_R_i_M was a mapped view onto y_R
		y_R.col(i) = y_R_i;
	}

	// storing strictly for diagnostic purposes, not used in algorithm.
	// TODO: move to struct Debug.
	Eigen::Index max_row, max_col;
	gaps->yabsmax = y.array().abs().maxCoeff(&max_row, &max_col);
	gaps->max_row = max_row;
	gaps->max_col = max_col;

	return true;
}

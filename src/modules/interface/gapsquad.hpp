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
	Vec g(0, 0, GRAV);
	Mat I3 = Mat::Identity();

	Vec up = x.R.col(2);
	Vec acc = u.thrust * up - g;

	Eigen::Matrix<FLOAT, 3, XDIM> Dacc_x;
	Dacc_x.setZero();
	// I3 wrt Z column of R
	Dacc_x.block<3, 3>(0, 3 + 3 + 3 + 6) = u.thrust * I3;

	// Normally I would use symplectic Euler integration, but plain forward
	// Euler gives simpler Jacobians.

	Mat hatw = hat(x.w);
	Mat exp_dt_hatw = I3 + dt * hatw + (dt * dt / 2) * hatw * hatw;

	Mat99 Dhatw2_hatw = kroneckerProduct(hatw.transpose(), I3) + kroneckerProduct(I3, hatw);
	Mat93 Dexp_w = dt * Dhat_w + (dt * dt / 2) * (Dhatw2_hatw * Dhat_w);

	x_t.ierr = x.ierr + dt * (x.p - t.p_d);
	x_t.p = x.p + dt * x.v;
	x_t.v = x.v + dt * acc;
	x_t.R = x.R * exp_dt_hatw;
	x_t.w = x.w + dt * u.torque;

	// TODO: This became trivial after we went from angle state to rotation
	// matrix -- condense some ops.
	Mat39 Dvt_R = dt * Dacc_x.block<3, 9>(0, 9);

	Mat99 DRt_R = kroneckerProduct(exp_dt_hatw.transpose(), I3);

	Vec Rx, Ry, Rz;
	colsplit(x.R, Rx, Ry, Rz);

	Mat93 DRt_w = kroneckerProduct(I3, x.R) * Dexp_w;

	// auto keeps the expression templates, for possible optimization
	auto Z33 = Mat::Zero();
	auto Z39 = Mat39::Zero();
	auto Z93 = Mat93::Zero();
	// auto Z91 = Eigen::Matrix<FLOAT, 9, 1, Eigen::RowMajor>::Zero();
	auto dt3 = dt * I3;

	Dx_x <<
		 I3, dt3, Z33,   Z39,   Z33,
		Z33,  I3, dt3,   Z39,   Z33,
		Z33, Z33,  I3, Dvt_R,   Z33,
		Z93, Z93, Z93, DRt_R, DRt_w,
		Z33, Z33, Z33,   Z39,    I3;
	// (Refers to Dx_x construction above.) Skipping Coriolis term that would
	// make dw'/dw nonzero because it requires system ID of the inertia matrix,
	// which we can otherwise skip. For the Crazyflie this term can be
	// neglected as the quad's inertia is very small.

	Dx_u.setZero();
	Dx_u.block<3, 1>(6, 0) = dt * Rz;
	Dx_u.block<3, 3>(9 + 9, 1) = dt3;
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
	VecT perr = (x.p - t.p_d).transpose();
	VecT verr = (x.v - t.v_d).transpose();
	VecT werr = (x.w - t.w_d).transpose();

	// It doesn't make sense to have a cost on the attitude because the target
	// doesn't include a specific target attitude. The only fully interpretable
	// errors are tracking (position) and "jitteriness" (omega and/or torque).
	// Velocity tracking is also marginal to penalize...

	c = (FLOAT)0.5 * (
		Q.p * perr.squaredNorm()
		+ Q.v * verr.squaredNorm()
		+ Q.w * werr.squaredNorm()
		+ Q.thrust * (u.thrust * u.thrust)
		+ Q.torque * u.torque.squaredNorm()
	);
	Dc_x <<
		Eigen::Matrix<FLOAT, 1, 3>::Zero(),
		Q.p * perr,
		Q.v * verr,
		Eigen::Matrix<FLOAT, 1, 9>::Zero(),
		Q.w * werr;
	Dc_u <<
		Q.thrust * u.thrust,
		Q.torque * u.torque.transpose();
}

// spiritually these are function-static to ctrl().
// see comment for gaps_step().
static Eigen::Matrix<FLOAT, 3, XDIM> Da_x;
static Eigen::Matrix<FLOAT, 3, TDIM> Da_th;
static Eigen::Matrix<FLOAT, 9, 3> DRd_a;
static Mat39 Der_R, Der_Rd;
static Eigen::Matrix<FLOAT, 3, XDIM> Der_x;
static Eigen::Matrix<FLOAT, 3, TDIM> Der_th;
static Eigen::Matrix<FLOAT, 3, XDIM> Dtorque_x;
static Eigen::Matrix<FLOAT, 3, TDIM> Dtorque_th;
static Eigen::Matrix<FLOAT, 1, XDIM> Dthrust_xRpart;

void ctrl(
	State const &x, Target const &t, Param const &th, // inputs
	Action &u, Jux &Du_x, Jut &Du_th, // main outputs
	Debug &debug,
	FLOAT dt // params
	)
{
	Vec const g(0, 0, GRAV);
	Mat const I = Mat::Identity();

	Diag const ki(th.ki_xy, th.ki_xy, th.ki_z);
	Diag const kp(th.kp_xy, th.kp_xy, th.kp_z);
	Diag const kv(th.kv_xy, th.kv_xy, th.kv_z);
	Diag const kr(th.kr_xy, th.kr_xy, th.kr_z);
	Diag const kw(th.kw_xy, th.kw_xy, th.kw_z);

	// position part components
	Vec const perr = x.p - t.p_d;
	Vec const verr = x.v - t.v_d;
	// Parens because Eigen forbids negating diagonal matrices for some reason?
	Vec const feedback = - (ki * x.ierr) - (kp * perr) - (kv * verr);
	Vec const a = feedback + t.a_d + g;

	Da_x << -(ki * I), -(kp * I), -(kv * I), Eigen::Matrix<FLOAT, 3, 9 + 3>::Zero();

	Da_th <<
		-x.ierr[0],          0, -perr[0],        0, -verr[0],        0, 0, 0, 0, 0,
		-x.ierr[1],          0, -perr[1],        0, -verr[1],        0, 0, 0, 0, 0,
		         0, -x.ierr[2],        0, -perr[2],        0, -verr[2], 0, 0, 0, 0;

	Vec const Rz = x.R.col(2);
	u.thrust = a.dot(Rz);
	VecT const Dthrust_a = Rz.transpose();
	Dthrust_xRpart <<
		Eigen::Matrix<FLOAT, 1, 3 + 3 + 3 + 3 + 3>::Zero(), a.transpose(), VecT::Zero();
	//                          i   p   v  Rx  Ry

	Vec zgoal;
	Mat Dzgoal_a;
	// Note this threshold is much higher than necessary for numerical
	// purposes, but since `a` should be 9.81 for gravity compensation,
	// magnitude of 0.1 is effectively very close to free fall. (Possibly below
	// the minimum motor speed for many quads.) We can therefore assume that
	// the direction of a is not particularly meaningful and may not be stable
	// over time. In such cases, it is probably not productive to track it.
	if (a.norm() > 0.1f) {
		zgoal = normalize(a, Dzgoal_a);
	}
	else {
		// Pos controller wants to free fall. This controller isn't really
		// designed for such aggressive maneuvers, but it seems reasonable to
		// fall in a straight and level attitude so all directions of lateral
		// (x-y) thrust are equally easy to achieve in the future.
		zgoal = Vec(0, 0, 1);
		Dzgoal_a.setZero();
	}
	debug.z_axis_desired = zgoal;

	Vec const xgoalflat(std::cos(t.y_d), std::sin(t.y_d), 0);
	Mat Dygoalnn_zgoal, dummy;
	Vec const ygoalnn = cross(zgoal, xgoalflat, Dygoalnn_zgoal, dummy);
	Mat Dygoal_ygoalnn;
	// TODO: handle case where too close to zero.
	Vec const ygoal = normalize(ygoalnn, Dygoal_ygoalnn);
	Mat Dygoal_a = Dygoal_ygoalnn * Dygoalnn_zgoal * Dzgoal_a;

	Mat Dxgoal_ygoal, Dxgoal_zgoal;
	Vec const xgoal = cross(ygoal, zgoal, Dxgoal_ygoal, Dxgoal_zgoal);
	Mat const Dxgoal_a = Dxgoal_ygoal * Dygoal_a + Dxgoal_zgoal * Dzgoal_a;
	Mat const Rd = fromcols(xgoal, ygoal, zgoal);

	#ifndef CRAZYFLIE_FW
	{
		// extra correctness checks
		FLOAT norm = xgoal.norm();
		if (std::abs(norm - 1) > 1e-6) {
			throw std::runtime_error("xgoal norm too far from 1: is " + std::to_string(norm));
		}
		FLOAT det = Rd.determinant();
		if (std::abs(det - 1) > 1e-6) {
			throw std::runtime_error("Rd determinant too far from 1: is " + std::to_string(det));
		}
		Mat RdTRd = Rd.transpose() * Rd;
		FLOAT maxerr = (RdTRd - I).array().abs().maxCoeff();
		if (maxerr > 1e-6) {
			throw std::runtime_error("Rd is not orthogonal: maxerr is " + std::to_string(maxerr));
		}
	}
	#endif

	DRd_a.block<3, 3>(0, 0) = Dxgoal_a;
	DRd_a.block<3, 3>(3, 0) = Dygoal_a;
	DRd_a.block<3, 3>(6, 0) = Dzgoal_a;

	Vec const er = SO3error(x.R, Rd, Der_R, Der_Rd);
	Vec const ew = x.w - t.w_d;
	debug.eR = er;
	debug.ew = ew;

	Vec dwerr = (1.0f / dt) * (ew - x.werr);
	dwerr[2] = 0.0f;

	Arr3 const dw_raw = 10 * (-(kr * er) - (kw * ew) - (th.kdw_xy * dwerr));
	#define RP_LIM 268
	#define Y_LIM 56
	Arr3 const dw_lims(RP_LIM, RP_LIM, Y_LIM);
	Arr3 const dw = dw_lims * (dw_raw / dw_lims).tanh();
	u.torque = dw.matrix();
	Diag const Ddw_dwraw =
		(dw_raw.array() / dw_lims).cosh().square().inverse().matrix().asDiagonal();

	// controller chain rules
	auto const Dthrust_x = Dthrust_a * Da_x + Dthrust_xRpart;
	auto const Dthrust_th = Dthrust_a * Da_th;

	Der_x.setZero();
	Der_x.block<3, 9>(0, 9) = Der_R;
	Der_x += Der_Rd * DRd_a * Da_x;

	Der_th = Der_Rd * DRd_a * Da_th;

	Dtorque_x = -(kr * Der_x);
	Dtorque_x.block<3, 3>(0, 3 + 3 + 3 + 9) -= kw * I;
	Dtorque_x = 10 * Ddw_dwraw * Dtorque_x;

	Dtorque_th = -(kr * Der_th); // indirect part
	Dtorque_th += (Eigen::Matrix<FLOAT, 3, TDIM>() <<
		0, 0, 0, 0, 0, 0, -er[0],      0, -ew[0],      0,
		0, 0, 0, 0, 0, 0, -er[1],      0, -ew[1],      0,
		0, 0, 0, 0, 0, 0,      0, -er[2],      0, -ew[2]).finished();
	Dtorque_th = 10 * Ddw_dwraw * Dtorque_th;

	Du_x << Dthrust_x, Dtorque_x;
	Du_th << Dthrust_th, Dtorque_th;
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

	gaps->prev_w_err = x->w - t->w_d;

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

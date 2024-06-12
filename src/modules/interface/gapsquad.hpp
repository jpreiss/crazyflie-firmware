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
using MapTheta = Eigen::Map<Theta>;
using X = Eigen::Matrix<FLOAT, XDIM, 1>;
static_assert(sizeof(X) == sizeof(State));
using MapX = Eigen::Map<X>;
using U = Eigen::Matrix<FLOAT, UDIM, 1>;
static_assert(sizeof(U) == sizeof(Action));
using MapU = Eigen::Map<U>;
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


void dynamics(
	State const &x, Target const &t, Action const &u, FLOAT const dt, // inputs
	State &x_t, Jxx &Dx_x, Jxu &Dx_u // outputs
	)
{
	X xt;
	Eigen::Matrix<FLOAT, XDIM, XDIM + UDIM> D;
	sym::Dynamics<FLOAT>(
		x.ierr, x.p, x.v, x.logR, x.w,
		t.p_d,
		u.thrust, u.torque,
		dt,
		&xt, &D
	);
	//using Rmap = Eigen::Map<Eigen::Matrix<FLOAT, 3, 3, Eigen::ColMajor> >;
	x_t.ierr = xt.head<3>();
	x_t.p = xt.segment<3>(3);
	x_t.v = xt.segment<3>(6),
	x_t.logR = xt.segment<3>(9);
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

// pybind11 can't modify python's immutable primitives (e.g. floats) by
// reference, so we return `c`. previously it was only an output mutable
// reference for consistency with other outputs. other callers besides pybind11
// still use the output ref, so for now we can have both.
float cost(
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
	return c;
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
		x.ierr, x.p, x.v, x.logR, x.w,
		t.p_d, t.v_d, t.a_d,
		t.w_d,
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

static Theta random_spherical()
{
	// central limit theorem - approx normal RV by sum of uniform RVs.
	// Eigen doesn't have normal RVs.
	Theta th = Theta::Random() + Theta::Random() + Theta::Random() + Theta::Random();
	th /= th.matrix().norm();
	return th;
}


Theta single_point_update(SinglePointGrad &sp, FLOAT eta, FLOAT cost)
{
	sp.cost_accum += cost;
	++sp.ep_step;
	if (sp.ep_step >= sp.ep_len) {
		DEBUG_PRINT("Single point update.\n");
		// use priveleged knowledge that tracking error with detuned parameters
		// on the aggressive diagonal figure-8 is around 15cm on average.
		sp.cost_accum -= (FLOAT)(0.15 * 0.15) * sp.ep_len;
		MapTheta perturbation(sp.perturbation);
		// "undo" the initial perturbation
		Theta update = -perturbation;
		// do the gradient descent approximation
		update -= eta * (TDIM / sp.radius) * sp.cost_accum * perturbation;
		// sample a new perturbation
		perturbation = sp.radius * random_spherical();
		// enact the new perturbation on the live parameters
		update += perturbation;
		sp.ep_step = 0;
		sp.cost_accum = 0;
		return update;
	}
	return Theta::Zero();
}

static void actor_critic_update(
	ActorCriticLSVI &ac, Param &theta, FLOAT eta,
	State const &x, Target const &t, FLOAT const cost,
	Jxx const &Dx_x, Jxu const &Dx_u,
	Jux const &Du_x, Jut const &Du_t,
	Gcx const &Dc_x, Gcu const &Dc_u
)
{
	// value is quadratic form in state error, not state
	State xerr_s = x;
	xerr_s.p -= t.p_d;
	xerr_s.v -= t.v_d;
	xerr_s.logR.setZero();
	xerr_s.w -= t.w_d;

	MapX xerr((FLOAT *)&xerr_s);
	MapX xerrprev((FLOAT *)&ac.xerrprev);

	Eigen::Map<Jxx> V(ac.V[0]);
	Eigen::Map<Jxx> Vtarget(ac.Vtarget[0]);
	Eigen::Map<GapsY> Dx_t_prev(ac.Dx_t_prev[0]);
	MapTheta Dc_t_prev(ac.Dc_t_prev);

	if (!ac.init) {
		V.setZero();
		Vtarget = V;
		ac.xerrprev = xerr_s;
		ac.costprev = cost;
		ac.vprev = 0;
		Dc_t_prev = Dc_u * Du_t;
		Dx_t_prev = Dx_u * Du_t;
		ac.init = true;
		return;
	}

	// critic update
	// Least-Squares Value Iteration update, i.e. gradient descent w.r.t. phi on
	// 1/2 ( V_phi_fixed(x') + r(x, u) - V_phi(x) )^2
	FLOAT Vx = xerr.transpose() * Vtarget * xerr;
	FLOAT actual = ac.costprev + ac.gamma * Vx;
	auto xxT = xerrprev * xerrprev.transpose();
	V += ac.critic_rate * (actual - ac.vprev) * xxT;
	Vtarget += ac.target_rate * (V - Vtarget);

	// actor update
	auto Dv_x = (2 * V * xerr).transpose();
	auto Dq_t = Dv_x * Dx_t_prev + Dc_t_prev.matrix().transpose();
	/*
	std::cout << "Dv_x = " << Dv_x << "\n";
	std::cout << "Dx_u = " << Dx_u << "\n";
	std::cout << "Dc_u = " << Dc_u << "\n";
	std::cout << "Du_t = " << Du_t << "\n";
	std::cout << "Dq_t = " << Dq_t << "\n";
	std::cout << " eta = " << eta << "\n";
	*/

	MapTheta((FLOAT *)&theta) -= eta * Dq_t.array();

	ac.xerrprev = xerr_s;
	ac.vprev = xerr.transpose() * V * xerr;
	ac.costprev = cost;
	Dc_t_prev = Dc_u * Du_t;
	Dx_t_prev = Dx_u * Du_t;
}


extern "C" bool gaps_step(
	struct GAPS *gaps,
	struct State const *x,
	struct Target const *t,
	FLOAT const dt,
	struct Action *u_out)
{
	gaps_optimizer const opt = (gaps_optimizer)gaps->optimizer;

	ctrl(*x, *t, gaps->theta, *u_out, Du_x, Du_t, gaps->debug, dt);
	// std::cout << "u = " << MapU((FLOAT *)u_out) << "\n";

	// integrate the ierr right away in case we exit due to being disabled.
	gaps->ierr += dt * (x->p - t->p_d);
	gaps->ierr[0] = clampsym(gaps->ierr[0], (FLOAT)2.0);
	gaps->ierr[1] = clampsym(gaps->ierr[1], (FLOAT)2.0);
	gaps->ierr[2] = clampsym(gaps->ierr[2], (FLOAT)0.4);

	FLOAT stage_cost;
	cost(*x, *t, *u_out, gaps->cost_param, stage_cost, Dc_x, Dc_u);
	gaps->sum_cost += dt * stage_cost;

	if (!gaps->enable) {
		return true;
	}

	MapTheta theta((FLOAT *)&gaps->theta);

	if (opt == GAPS_OPT_SINGLEPOINT) {
		theta += single_point_update(gaps->single_point, gaps->eta, stage_cost);
		return true;
	}
	
	dynamics(*x, *t, *u_out, dt, xnext, Dx_x, Dx_u);

	if (opt == GAPS_OPT_ACTORCRITIC) {
		actor_critic_update(
			gaps->actor_critic, gaps->theta, gaps->eta,
			*x, *t, stage_cost,
			Dx_x, Dx_u, Du_x, Du_t, Dc_x, Dc_u
		);
		return true;
	}

	// 1) compute the gradient
	Eigen::Map<GapsY> y(gaps->y[0]);
	Theta grad = (Dc_x + (Dc_u * Du_x) * y + Dc_u * Du_t).array();
	// regularization
	grad += gaps->cost_param.reg_L2 * theta;

	// 2) run the optimizer
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
	else if (opt == GAPS_OPT_EPISODIC_GRAD) {
		EpisodicGrad &eg = gaps->episodic_grad;
		MapTheta accum(eg.grad_accum);
		accum += grad;
		++eg.ep_step;
		if (eg.ep_step >= eg.ep_len) {
			DEBUG_PRINT("Episodic grad update.\n");
			theta -= gaps->eta * accum;
			accum.setZero();
			y.setZero();
			eg.ep_step = 0;
		}
	}
	else {
		return false;
	}

	// GAPS 3) update the state
	// no projection due to log-space
	// dynamic programming (with slight damping)
	auto product = Dx_x + Dx_u * Du_x;
	// std::cout << "product eigvals:" << product.eigenvalues() << "\n";
	y = gaps->damping * product * y + Dx_u * Du_t;

	// Storing strictly for diagnostic purposes, not used in algorithm.
	// Stored in 1-based indexing so zero can serve as emptiness sentinel.
	// TODO: move to struct Debug.
	Eigen::Index max_row, max_col;
	gaps->yabsmax = y.array().abs().maxCoeff(&max_row, &max_col);
	gaps->max_row = max_row + 1;
	gaps->max_col = max_col + 1;

	return true;
}

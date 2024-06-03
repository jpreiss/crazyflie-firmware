#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

using FLOAT = double;
#include "gapsquad.hpp"


using StateTuple = std::tuple<Vec, Vec, Vec, Vec, Vec>;
using ActionTuple = std::tuple<FLOAT, Vec>;
using TargetTuple = std::tuple<Vec, Vec, Vec, Vec>;
using ParamTuple = std::tuple<
	FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, // position gains
	FLOAT, FLOAT, FLOAT, FLOAT // attitude gains
>;
using CostParamTuple = std::tuple<FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT>;


// TODO: Figure out how to use variadic macros to replace this boilerplate.

State t2s(StateTuple const &st)
{
	State s;
	std::tie(s.ierr, s.p, s.v, s.logR, s.w) = st;
	return s;
}
StateTuple s2t(State const &s)
{
	return std::make_tuple(s.ierr, s.p, s.v, s.logR, s.w);
}

Target t2s(TargetTuple const &tt)
{
	Target t;
	std::tie(t.p_d, t.v_d, t.a_d, t.w_d) = tt;
	return t;
}
TargetTuple s2t(Target const &t)
{
	return std::make_tuple(t.p_d, t.v_d, t.a_d, t.w_d);
}

Action t2s(ActionTuple const &at)
{
	Action a;
	std::tie(a.thrust, a.torque) = at;
	return a;
}
ActionTuple s2t(Action const &a)
{
	return std::make_tuple(a.thrust, a.torque);
}

Param t2s(ParamTuple const &pt)
{
	Param p;
	std::tie(
		p.ki_xy, p.ki_z,
		p.kp_xy, p.kp_z,
		p.kv_xy, p.kv_z,
		p.kr_xy, p.kr_z,
		p.kw_xy, p.kw_z) = pt;
	return p;
}
ParamTuple s2t(Param const &p)
{
	return std::make_tuple(
		p.ki_xy, p.ki_z,
		p.kp_xy, p.kp_z,
		p.kv_xy, p.kv_z,
		p.kr_xy, p.kr_z,
		p.kw_xy, p.kw_z);
}

CostParam t2s(CostParamTuple const &cpt)
{
	CostParam cp;
	std::tie(cp.p, cp.v, cp.w, cp.thrust, cp.torque, cp.reg_L2) = cpt;
	return cp;
}
CostParamTuple s2t(CostParam const &cp)
{
	return std::make_tuple(cp.p, cp.v, cp.w, cp.thrust, cp.torque, cp.reg_L2);
}

std::tuple<ActionTuple, Jux, Jut>
ctrl_wrap(StateTuple const &xt, TargetTuple const &tt, ParamTuple const &tht, FLOAT dt)
{
	std::tuple<ActionTuple, Jux, Jut> output;
	Action u;
	Debug debug;
	ctrl(t2s(xt), t2s(tt), t2s(tht), u, std::get<Jux>(output), std::get<Jut>(output), debug, dt);
	std::get<ActionTuple>(output) = s2t(u);
	return output;
}

std::tuple<StateTuple, Jxx, Jxu>
dynamics_wrap(StateTuple const &xt, TargetTuple const &tt, ActionTuple const &ut, FLOAT dt)
{
	std::tuple<StateTuple, Jxx, Jxu> output;
	State xnext;
	dynamics(t2s(xt), t2s(tt), t2s(ut), dt, xnext, std::get<Jxx>(output), std::get<Jxu>(output));
	std::get<StateTuple>(output) = s2t(xnext);
	return output;
}

std::tuple<FLOAT, Gcx, Gcu>
cost_wrap(StateTuple const &xt, TargetTuple const &tt, ActionTuple const &ut, CostParamTuple const &Qt) // inputs
{
	std::tuple<FLOAT, Gcx, Gcu> output;
	cost(t2s(xt), t2s(tt), t2s(ut), t2s(Qt),
		std::get<FLOAT>(output), std::get<Gcx>(output), std::get<Gcu>(output));
	return output;
}

namespace py = pybind11;

// actorcritic extensions
namespace ac {
	Jxx get_V(ActorCriticLSVI const *ac)
	{
		return Eigen::Map<Jxx const>(&ac->V[0][0]);
	}
	void set_V(ActorCriticLSVI *ac, Jxx V)
	{
		Eigen::Map<Jxx>(&ac->V[0][0]) = V;
	}
	Jxx get_Vtarget(ActorCriticLSVI const *ac)
	{
		return Eigen::Map<Jxx const>(&ac->Vtarget[0][0]);
	}
	void set_Vtarget(ActorCriticLSVI *ac, Jxx Vtarget)
	{
		Eigen::Map<Jxx>(&ac->Vtarget[0][0]) = Vtarget;
	}
	StateTuple get_xerrprev(ActorCriticLSVI const *ac)
	{
		return s2t(ac->xerrprev);
	}
	ParamTuple update_wrap(
		ActorCriticLSVI *ac, ParamTuple const &pt, FLOAT eta,
		StateTuple const &xt, TargetTuple const &xdt, FLOAT cost,
		Jxx const &Dx_x, Jxu const &Dx_u,
		Jux const &Du_x, Jut const &Du_t,
		Gcx const &Dc_x, Gcu const &Dc_u)
	{
		Param theta = t2s(pt);
		actor_critic_update(
			*ac, theta, eta,
			t2s(xt), t2s(xdt), cost,
			Dx_x, Dx_u, Du_x, Du_t, Dc_x, Dc_u);
		return s2t(theta);
	}
};

PYBIND11_MODULE(gapsquad, m) {
	m.def("ctrl", &ctrl_wrap);
	m.def("dynamics", &dynamics_wrap);
	m.def("cost", &cost_wrap);

	m.def("ctrl_s", &ctrl);
	m.def("dynamics_s", &dynamics);
	m.def("cost_s", &cost);

	m.attr("XDIM") = py::cast(XDIM);
	m.attr("UDIM") = py::cast(UDIM);
	m.attr("TDIM") = py::cast(TDIM);

	py::class_<ActorCriticLSVI>(m, "ActorCriticLSVI")
		.def(py::init<>())
		.def("update", &ac::update_wrap)
		.def_property("V", &ac::get_V, &ac::set_V)
		.def_property("Vtarget", &ac::get_Vtarget, &ac::set_Vtarget)
		.def_property_readonly("xerrprev", &ac::get_xerrprev)
		.def_readwrite("init", &ActorCriticLSVI::init)
		.def_readwrite("vprev", &ActorCriticLSVI::vprev)
		.def_readwrite("costprev", &ActorCriticLSVI::costprev)
		.def_readwrite("critic_rate", &ActorCriticLSVI::critic_rate)
		.def_readwrite("target_rate", &ActorCriticLSVI::target_rate)
		.def_readwrite("gamma", &ActorCriticLSVI::gamma);
}

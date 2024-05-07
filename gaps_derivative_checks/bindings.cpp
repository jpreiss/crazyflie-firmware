#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

using FLOAT = double;
#include "gapsquad.hpp"


using StateTuple = std::tuple<Vec, Vec, Vec, Mat, Vec>;
using ActionTuple = std::tuple<FLOAT, Vec>;
using TargetTuple = std::tuple<Vec, Vec, Vec, FLOAT, Vec>;
using ParamTuple = std::tuple<
	FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, // position gains
	FLOAT, FLOAT, FLOAT, FLOAT // attitude gains
>;
using CostParamTuple = std::tuple<FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT>;


// TODO: Figure out how to use variadic macros to replace this boilerplate.

State t2s(StateTuple const &st)
{
	State s;
	std::tie(s.ierr, s.p, s.v, s.R, s.w) = st;
	return s;
}
StateTuple s2t(State const &s)
{
	return std::make_tuple(s.ierr, s.p, s.v, s.R, s.w);
}

Target t2s(TargetTuple const &tt)
{
	Target t;
	std::tie(t.p_d, t.v_d, t.a_d, t.y_d, t.w_d) = tt;
	return t;
}
TargetTuple s2t(Target const &t)
{
	return std::make_tuple(t.p_d, t.v_d, t.a_d, t.y_d, t.w_d);
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
ctrl_wrap(StateTuple const &xt, TargetTuple const &tt, ParamTuple const &tht)
{
	std::tuple<ActionTuple, Jux, Jut> output;
	Action u;
	ctrl(t2s(xt), t2s(tt), t2s(tht), u, std::get<Jux>(output), std::get<Jut>(output));
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

PYBIND11_MODULE(gapsquad, m) {
	// m.def("angleto", &angleto);
	m.def("ctrl", &ctrl_wrap);
	m.def("dynamics", &dynamics_wrap);
	m.def("cost", &cost_wrap);
	m.def("SO3error", &SO3error);
	m.def("cross", &cross);
	m.def("hat", &hat);
	m.def("normalize", &normalize);
}

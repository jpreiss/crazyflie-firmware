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

Jxx actor_critic_get_V(ActorCriticLSVI const *ac)
{
	return Eigen::Map<Jxx const>(&ac->V[0][0]);
}
void actor_critic_set_V(ActorCriticLSVI *ac, Jxx V)
{
	Eigen::Map<Jxx>(&ac->V[0][0]) = V;
}

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

	// sigh... pybind11 or SWIG... pick your poison
	py::class_<State>(m, "State")
		.def(py::init<>())
		.def_readwrite("ierr", &State::ierr)
		.def_readwrite("p", &State::p)
		.def_readwrite("v", &State::v)
		.def_readwrite("logR", &State::logR)
		.def_readwrite("w", &State::w);

	py::class_<Action>(m, "Action")
		.def(py::init<>())
		.def_readwrite("thrust", &Action::thrust)
		.def_readwrite("torque", &Action::torque);

	py::class_<Target>(m, "Target")
		.def(py::init<>())
		.def("__copy__",  [](Target const xd) { return xd; })
		.def_readwrite("p_d", &Target::p_d)
		.def_readwrite("v_d", &Target::v_d)
		.def_readwrite("a_d", &Target::a_d)
		.def_readwrite("w_d", &Target::w_d);

	py::class_<Param>(m, "Param")
		.def(py::init<>())
		.def("__copy__",  [](Param const t) { return t; })
		.def_readwrite("ki_xy", &Param::ki_xy).def_readwrite("ki_z", &Param::ki_z)
		.def_readwrite("kp_xy", &Param::kp_xy).def_readwrite("kp_z", &Param::kp_z)
		.def_readwrite("kv_xy", &Param::kv_xy).def_readwrite("kv_z", &Param::kv_z)
		.def_readwrite("kr_xy", &Param::kr_xy).def_readwrite("kr_z", &Param::kr_z)
		.def_readwrite("kw_xy", &Param::kw_xy).def_readwrite("kw_z", &Param::kw_z);

	py::class_<CostParam>(m, "CostParam")
		.def(py::init<>())
		.def_readwrite("p", &CostParam::p)
		.def_readwrite("v", &CostParam::v)
		.def_readwrite("w", &CostParam::w)
		.def_readwrite("thrust", &CostParam::thrust)
		.def_readwrite("torque", &CostParam::torque)
		.def_readwrite("reg_L2", &CostParam::reg_L2);

	py::class_<Debug>(m, "Debug")
		.def(py::init<>())
		.def_readwrite("z_axis_desired", &Debug::z_axis_desired)
		.def_readwrite("eR", &Debug::eR)
		.def_readwrite("ew", &Debug::ew)
		.def_readwrite("dw_squash", &Debug::dw_squash);

	py::class_<ActorCriticLSVI>(m, "ActorCriticLSVI")
		.def(py::init<>())
		.def_property("V", &actor_critic_get_V, &actor_critic_set_V)
		.def_readwrite("init", &ActorCriticLSVI::init)
		.def_readwrite("xerrprev", &ActorCriticLSVI::xerrprev)
		.def_readwrite("vprev", &ActorCriticLSVI::vprev)
		.def_readwrite("costprev", &ActorCriticLSVI::costprev)
		.def_readwrite("critic_rate", &ActorCriticLSVI::critic_rate)
		.def_readwrite("gamma", &ActorCriticLSVI::gamma);
	
	m.def("actor_critic_update", &actor_critic_update);

}

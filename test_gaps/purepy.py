import itertools as it

import numpy as np

import gapsquad
from gapsquad import SO3error, cross, hat, normalize
import SO3

import symforce
import symforce.symbolic as sf
import sympy as sym

import codegen
from testlib import *



def normalize(x):
    return x / x.norm()


def ctrl_py(x: State, xd: Target, th: Param, dt: float):
    """Returns: u, Du_x, Du_th."""

    R = x.R.reshape(3, 3).T
    outputs = codegen.ctrl(
        x.ierr, x.p, x.v, R, x.w,
        xd.p_d, xd.v_d, xd.a_d, xd.y_d, xd.w_d,
        th.to_arr()[:6], th.to_arr()[6:],
        dt
    )
    thrust, torque = outputs[:2]
    u = Action(thrust=thrust, torque=torque)

    outputs = outputs[2:]
    # remaining outputs are Jacobian blocks, but SymForce gives 1D vectors for
    # Jacobian blocks of scalars, we must raise to 2D for np.block().
    outputs = np.array(outputs, dtype=object).reshape((2, 7))
    for i in range(outputs.size):
        outputs.flat[i] = np.atleast_2d(outputs.flat[i])
    outputs = outputs.tolist()
    J = np.block(outputs)
    assert J.shape[0] == 4
    Du_x = J[:, :21]
    Du_th = J[:, 21:]
    assert Du_th.shape[-1] == 10
    return u, Du_x, Du_th


def dynamics_py(x: State, xd: Target, u: Action, dt: float):
    """Returns: x, Dx_x, Dx_u."""

    R = x.R.reshape(3, 3).T
    outputs = codegen.dynamics(
        x.ierr, x.p, x.v, R, x.w, xd.p_d, u.thrust, u.torque, dt)

    ierrt, pt, vt, Rt, wt = outputs[:5]
    x_t = State(ierrt, pt, vt, Rt.T.flatten(), wt)
    outputs = outputs[5:]

    # remaining outputs are Jacobian blocks, but SymForce gives 1D vectors for
    # Jacobian blocks of scalars, we must raise to 2D for np.block().
    outputs = np.array(outputs, dtype=object).reshape((5, 7))
    for i in range(outputs.size):
        outputs.flat[i] = np.atleast_2d(outputs.flat[i].T).T
    outputs = outputs.tolist()
    J = np.block(outputs)
    assert J.shape[0] == 21
    Dx_x = J[:, :21]
    Dx_u = J[:, 21:]
    assert Dx_u.shape[-1] == 4
    return x_t, Dx_x, Dx_u


def cost_py(x: State, xd: Target, u: Action, Q: CostParam):
    """Returns: c, Dc_x, Dc_u."""

    outputs = codegen.cost(
        x.p, x.v, x.w,
        xd.p_d, xd.v_d, xd.w_d,
        u.thrust, u.torque,
        Q.p, Q.v, Q.w, Q.thrust, Q.torque)

    c = outputs[0]
    for o in outputs[1:]:
        assert len(o.shape) == 1
    J = np.concatenate(outputs[1:])
    Dc_x = np.zeros((1, 21))
    # no ierr
    Dc_x[:, 3:9] = J[0:6]
    # no rot
    Dc_x[:, 18:] = J[6:9]
    Dc_u = J[9:][None, ]
    assert Dc_u.size == 4

    return c, Dc_x, Dc_u

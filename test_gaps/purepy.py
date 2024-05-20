import itertools as it

import numpy as np

import gapsquad
#from gapsquad import SO3error, cross, hat, normalize
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

    u_a, D = codegen.ctrl(
        x.ierr, x.p, x.v, x.logR, x.w,
        xd.p_d, xd.v_d, xd.a_d, xd.y_d, xd.w_d,
        th.to_arr()[:6], th.to_arr()[6:],
        dt
    )
    u = Action.from_arr(u_a)
    Du_x = D[:, :15]
    Du_th = D[:, 15:]
    assert Du_th.shape[-1] == 10
    return u, Du_x, Du_th


def dynamics_py(x: State, xd: Target, u: Action, dt: float):
    """Returns: x, Dx_x, Dx_u."""

    x_t_a, D = codegen.dynamics(
        x.ierr, x.p, x.v, x.logR, x.w, xd.p_d, u.thrust, u.torque, dt)

    x_t = State.from_arr(x_t_a)
    Dx_x = D[:, :15]
    Dx_u = D[:, 15:]
    assert Dx_u.shape[-1] == 4
    return x_t, Dx_x, Dx_u


def cost_py(x: State, xd: Target, u: Action, Q: CostParam):
    """Returns: c, Dc_x, Dc_u."""

    c, D = codegen.cost(
        x.p, x.v, x.w,
        xd.p_d, xd.v_d, xd.w_d,
        u.thrust, u.torque,
        Q.p, Q.v, Q.w, Q.thrust, Q.torque)

    assert len(D.shape) == 1
    Dc_x = np.zeros((1, 15))
    # no ierr
    Dc_x[:, 3:9] = D[0:6]
    # no rot
    Dc_x[:, 12:] = D[6:9]
    Dc_u = D[None, 9:]
    assert Dc_u.shape[-1] == 4
    return c, Dc_x, Dc_u

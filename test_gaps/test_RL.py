from copy import copy, deepcopy
import itertools as it

#import matplotlib.pyplot as plt
import numpy as np
import pytest
#import scipy as sp

from gapsquad import *
#from testlib import *


# TODO: figure out how to make pybind11 bundle these
THETAS = [f"k{x}_{ax}" for x, ax in it.product("ipvrw", ["xy", "z"])]
State.__repr__ = lambda x: (
    f"State(ierr={x.ierr}, p={x.p}, v={x.v}, logR={x.logR}, w={x.w})"
)
Target.__repr__ = lambda xd: (
    f"Target(p_d={xd.p_d}, v_d={xd.v_d}, a_d={xd.a_d}, w_d={xd.w_d})"
)
Action.__repr__ = lambda u: f"Action(thrust={u.thrust}, torque={u.torque})"
Param.__repr__ = lambda t: (
    "Param(" + ",".join(f"{k}={getattr(t, k)}" for k in THETAS) + ")"
)


Z3 = np.zeros(3)


def zero_state():
    s = State()
    s.ierr = Z3
    s.p = Z3
    s.v = Z3
    s.logR = Z3
    s.w = Z3
    return s


def zero_target():
    xd = Target()
    xd.p_d = Z3
    xd.v_d = Z3
    xd.a_d = Z3
    xd.w_d = Z3
    return xd


def AC():
    ac = ActorCriticLSVI()
    ac.init = False
    ac.V = np.zeros((XDIM, XDIM))
    ac.xerrprev = zero_state()
    ac.vprev = 0
    ac.costprev = 0
    ac.critic_rate = 0.1
    ac.gamma = 0.5
    return ac


def zero_jacobians():
    Dx_x = np.zeros((XDIM, XDIM))
    Dx_u = np.zeros((XDIM, UDIM))
    Du_x = np.zeros((UDIM, XDIM))
    Du_t = np.zeros((UDIM, TDIM))
    Dc_x = np.zeros((1, XDIM))
    Dc_u = np.zeros((1, UDIM))
    return Dx_x, Dx_u, Du_x, Du_t, Dc_x, Dc_u


def default_param():
    t = Param()
    # recall we are in log space
    t.ki_xy = t.ki_z = 1
    t.kp_xy = t.kp_z = 3
    t.kv_xy = t.kv_z = 1
    t.kr_xy = 4
    t.kr_z = 3
    t.kw_xy = 3
    t.kw_z = 2
    return t


def test_actorcritic_V():
    ac = AC()

    t = Param()
    x = zero_state()
    xd = zero_target()
    xd.p_d = [1, 0, 0]
    jacs = zero_jacobians()
    eta = 1e-2
    cost = 1.0

    # gamma is 0.5, cost is always 1. Therefore, true value fn is 2 for any
    # state. xerr is (0, 0, 0, 1, 0, ..., 0). So for quadratic form, we need
    # V[3, 3] = 2.  All other entries of V could be anything if initialized as
    # nonzero, but since we initialize with zero, we expect them to be zero.
    # theta doesn't matter.
    for _ in range(1000):
        actor_critic_update(ac, t, eta, x, xd, cost, *jacs)

    V = ac.V
    print(V)
    assert np.isclose(V[3, 3], 2)
    V[3, 3] = 0
    assert np.allclose(V, 0)


def test_actorcritic_thrust():
    ac = AC()

    debug = Debug()
    dt = 0.01

    theta = default_param()
    x = zero_state()
    x.p = [0, 0, -1]
    x2 = zero_state()
    u = Action()
    xd = zero_target()
    jacs = zero_jacobians()
    Dx_x, Dx_u, Du_x, Du_t, Dc_x, Dc_u = jacs
    eta = 1e-0
    cp = CostParam()
    cp.p = 1.0
    c_dummy = 0.0

    ki_z_init = theta.ki_z
    kp_z_init = theta.kp_z
    kv_z_init = theta.kv_z

    # Initial gains of exp(0) = 1 are very low.
    # Add a constant downward disturbance (like wrong mass estimate).
    # Position gains of z should go up.
    for i in range(1000):
        ctrl_s(x, xd, theta, u, Du_x, Du_t, debug, dt)
        c = cost_s(x, xd, u, cp, c_dummy, Dc_x, Dc_u)
        assert c != 0
        dynamics_s(x, xd, u, dt, x2, Dx_x, Dx_u)
        theta_old = copy(theta)
        actor_critic_update(ac, theta, eta, x, xd, c, *jacs)
        diff = theta - theta_old
        if i > 10:
            assert not np.all(diff == 0)
        print(f"cost = {c}")
        print(f"u = {u}")
        print(f"x = {x2}")
        print(f"theta = {theta}")
        x = x2
        print(ac.V)
        assert not np.any(np.isnan(ac.V.flat))


    V = ac.V
    print(V)
    assert False

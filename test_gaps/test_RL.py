from copy import copy, deepcopy
import itertools as it

import matplotlib.pyplot as plt
import numpy as np
import pytest
#import scipy as sp

from gapsquad import *
from testlib import *


Z3 = np.zeros(3)


def AC():
    ac = ActorCriticLSVI()
    ac.init = False
    ac.V = np.zeros((XDIM, XDIM))
    ac.vprev = 0
    ac.costprev = 0
    ac.critic_rate = 0.01
    ac.target_rate = 0.001
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


def test_actorcritic_V():
    ac = AC()
    # big learning rate because test is inherently stable, keep it fast
    ac.critic_rate *= 10
    ac.target_rate *= 10
    ac.gamma = 0.5
    x, xd, th, *_ = default_inputs()
    xd = xd._replace(p_d = np.array([1, 0, 0]))
    jacs = zero_jacobians()
    eta = 1e-2
    cost = 1.0

    # gamma is 0.5, cost is always 1. Therefore, true value fn is 2 for any
    # state. xerr is (0, 0, 0, 1, 0, ..., 0). So for quadratic form, we need
    # V[3, 3] = 2.  All other entries of V could be anything if initialized as
    # nonzero, but since we initialize with zero, we expect them to be zero.
    # theta doesn't matter.
    for _ in range(1000):
        ac.update(th, eta, x, xd, cost, *jacs)

    V = ac.V
    print(V)
    assert np.isclose(V[3, 3], 2, atol=1e-2)
    V[3, 3] = 0
    assert np.allclose(V, 0)


@pytest.mark.xfail
def test_actorcritic_thrust():
    ac = AC()
    ac.gamma = 0.9
    ac.critic_rate = 0.01
    x, xd, th, Q, u = default_inputs(detune=0.25)
    xd = xd._replace(p_d = np.array([0, 0, 1]))
    #jacs = zero_jacobians()
    #Dx_x, Dx_u, Du_x, Du_t, Dc_x, Dc_u = jacs
    eta = 1e-8
    dt = 1e-2

    th_init = deepcopy(th)
    ac.V = np.eye(XDIM)

    # a strong force pulling down
    disturbance_acc = np.array([0, 0, -5*dt])

    T = 10000
    Vmaxes = np.zeros(T)
    costs = np.zeros(T)
    zs = np.zeros(T)
    kp_zs = np.zeros(T)

    # Initial gains of exp(0) = 1 are very low.
    # Add a constant downward disturbance (like wrong mass estimate).
    # Position gains of z should go up.
    for i in range(T):
        u, Du_x, Du_t = ctrl(x, xd, th, dt)
        c, Dc_x, Dc_u = cost(x, xd, u, Q)
        assert c > 0
        x2, Dx_x, Dx_u = dynamics(x, xd, u, dt)
        th_old = th
        th = Param(*ac.update(
            th, eta, x, xd, c, Dx_x, Dx_u, Du_x, Du_t, Dc_x, Dc_u))
        # if i > 10:
        #     assert th_old != th
        x = State(*x2)
        x = x._replace(v=x.v + disturbance_acc)
        costs[i] = c
        Vmaxes[i] = np.amax(np.abs(ac.V.flat))
        zs[i] = x.p[2]
        kp_zs[i] = th.kp_z
        assert not np.any(np.isnan(ac.V.flat))
        # the check below is slow, so we should write a different test with
        # smaller T to check it
        #assert np.allclose(ac.V, ac.V.T)

    V = ac.V
    print(V)

    plt.plot(Vmaxes, label="$|V|_\\infty$")
    plt.plot(costs, label="cost")
    plt.plot(zs, label="z")
    plt.plot(kp_zs, label="kp_z")
    #plt.plot(np.cumsum(costs), label="cumsum(cost)")
    plt.legend()
    plt.show()

    # integral and velocity are more complicated... but kp is definitely too
    # low with this detune
    assert th.kp_z > th_init.kp_z


if __name__ == "__main__":
    test_actorcritic_thrust()

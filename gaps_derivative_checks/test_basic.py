import numpy as np
import scipy as sp

from testlib import *
import SO3


Z3 = np.zeros(3)
I3 = np.eye(3)


def test_cost_sanity():
    x, xd, _, Q, u = default_inputs()
    c, Dc_x, Dc_u = cost_cpp(x, xd, u, Q)
    assert np.isclose(c, 0)


def test_exp():
    rng = np.random.default_rng(0)
    scales = 10.0 ** np.arange(-4, 5)
    for _ in range(100):
        scale = rng.choice(scales)
        w = scale * rng.normal(size=3)
        W = SO3.hat(w)
        Rrodrigues = SO3.exp(W)
        Rexp = sp.linalg.expm(W)
        assert np.allclose(Rrodrigues, Rexp)


def test_dynamics_freefall():
    rng = np.random.default_rng(0)
    omegas = [Z3] + [rng.normal(size=3) for _ in range(100)]
    dt = 0.01

    for w in omegas:
        x, xd, _, _, u = default_inputs()
        x = x._replace(w=w)
        for _ in range(100):
            x, _, _ = dynamics_cpp(x, xd, u, dt)

        # rotation should have no effect unless we have thrust
        assert np.allclose(x.v, [0, 0, -9.81])

        # final attitude should be close to the closed form
        R = SO3.exp(SO3.hat(w))
        Rx = x.R.reshape((3, 3)).T
        # TODO: we should implement true Rodrigues exp map in dynamics_cpp to
        # reduce error.
        assert np.allclose(Rx, R, rtol=1e-3, atol=1e-3)




# def test_dynamics_rot():
#     dt = 1e-2
#     rng = np.random.default_rng(0)
#     for i in range(100):
#         axis = rng.normal(size=3)
#         axis /= np.linalg.norm(axis)
#         # make sure we go past a full rotation sometimes
#         angle = 10 * rng.normal()
#         R


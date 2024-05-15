import matplotlib.pyplot as plt
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


def ctrb(A, B):
    n, m = B.shape
    Ak = np.eye(n)
    G = np.zeros((n, n))
    mats = []
    for _ in range(n):
        mats.append(Ak @ B)
        Ak = A @ Ak
    C = np.concatenate(mats, axis=1)
    assert C.shape == (n, m * n)
    return C


def test_linearized_controllability():
    # check controllability of linearized system
    dt = 0.1
    x, xd, th, *_ = default_inputs()
    u = Action(thrust=9.81, torque=np.zeros(3))
    _, A, B = dynamics_cpp(x, xd, u, dt)
    C = ctrb(A, B)
    U, S, VT = np.linalg.svd(C)
    sigma_min = 1e-6
    if np.all(S > sigma_min):
        return

    fig, ax = plt.subplots()
    plot_x = np.arange(State.size) + 1
    for i in range(1, State.size + 1):
        if S[-i] > sigma_min:
            break
        ax.plot(plot_x, U[:, -i], linewidth=0, marker=".", label=str(i))
    ax.set(xlabel="state index", ylabel="value", xticks=plot_x)
    ax.grid(True)
    ax.legend(title="singular vec")
    fig.show()
    assert False

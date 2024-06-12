import matplotlib.pyplot as plt
import numpy as np
import pytest
import scipy as sp

from purepy import dynamics_py
from testlib import *
import SO3


Z3 = np.zeros(3)
I3 = np.eye(3)

HI_GAIN_THETA_POS = (1, 1, 10, 10, 5, 5)
HI_GAIN_THETA_ROT = (100, 20, 20, 2)
HI_GAIN_THETA = np.concatenate([HI_GAIN_THETA_POS, HI_GAIN_THETA_ROT])


def test_cost_sanity():
    x, xd, _, Q, u = default_inputs()
    c, Dc_x, Dc_u = cost_cpp(x, xd, u, Q)
    assert np.isclose(c, 0)


def test_dynamics_freefall():
    rng = np.random.default_rng(0)
    omegas = [Z3] + [rng.normal(size=3) / 2 for _ in range(100)]
    norms = np.linalg.norm(omegas, axis=1)
    # this would cause wraparound
    assert not np.any(norms > 3.14)
    dt = 0.01

    for w in omegas:
        x, xd, _, _, u = default_inputs()
        x = x._replace(w=w)
        for _ in range(100):
            x, _, _ = dynamics_cpp(x, xd, u, dt)
            #print(x.logR)

        # rotation should have no effect unless we have thrust
        assert np.allclose(x.v, [0, 0, -9.81])

        assert np.allclose(x.logR, w)


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

    fig, ax = plt.subplots(figsize=(8, 4))
    plot_x = np.arange(State.size) + 1
    for i in range(1, State.size + 1):
        if S[-i] > sigma_min:
            break
        ax.plot(plot_x, U[:, -i], linewidth=0, marker=".", markersize=12, label=str(i))
    ax.set(xlabel="state index", ylabel="value", xticks=plot_x)
    ax.grid(True, color=[0.8, 0.8, 0.8])
    ax.legend(title="singular vec\n(smallest first)")
    fig.savefig("controllability_singular_vectors.pdf")

    assert False, "would have returned if controllable"


def test_ctrl_signs():
    # tests that ctrl wants to point the thrust vector in the correct direction.
    Z3 = np.zeros(3)
    x = State(ierr=Z3, p=Z3, v=Z3, logR=Z3, w=Z3)
    pos_torques = [
        (( 1,  0, 0), ( 0,  1, 0)),
        ((-1,  0, 0), ( 0, -1, 0)),
        (( 0,  1, 0), (-1,  0, 0)),
        (( 0, -1, 0), ( 1,  0, 0)),
    ]
    for pos, torque in pos_torques:
        target = Target(p_d=pos, v_d=Z3, a_d=Z3, w_d=Z3)
        u, *_ = ctrl_cpp(x, target, HI_GAIN_THETA, dt=0.1)
        assert u.thrust > 8
        assert np.all(np.sign(torque) == np.sign(u.torque))



def test_stabilizing():
    dt = 1e-2
    # TODO: tune so we don't need 80 seconds - closedloop must be underdamped.
    T = int(80 / dt)
    Z3 = np.zeros(3)
    target = Target(p_d=Z3, v_d=Z3, a_d=Z3, w_d=Z3)
    rng = np.random.default_rng(0)
    def close(x):
        return (
            np.allclose(x.p, Z3, atol=1e-2)
            and np.allclose(x.v, Z3, atol=1e-3)
            and np.allclose(x.logR, Z3, atol=1e-4)
            and np.allclose(x.w, Z3, atol=1e-4)
        )
    for i in range(10):
        x0, *_ = random_inputs(rng)
        assert not close(x0)
        x = x0
        for t in range(T):
            # TODO: tune gains for testing better
            u, *_ = ctrl_cpp(x, target, HI_GAIN_THETA/2, dt)
            x, *_ = dynamics_cpp(x, target, u, dt)
        assert close(x)

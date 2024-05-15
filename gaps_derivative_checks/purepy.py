from collections import namedtuple

import sys

from colorama import Fore
import numpy as np

import gapsquad
from gapsquad import SO3error, cross, hat, normalize
import SO3


def ctrl_py(x: State, xd: Target, th: Param, c: Const):
    """Returns: u, Du_x, Du_th."""

    g = np.array([0, 0, c.g])

    # CONTROLLER

    # ----------
    # position part components
    I = np.eye(3)
    perr = x.p - xd.p_d
    verr = x.v - xd.v_d
    feedback = - th.ki * x.ierr - th.kp * perr - th.kv * verr
    a = feedback + xd.a_d + g
    Da_x = np.block([
        [-th.ki * I, -th.kp * I, -th.kv * I, np.zeros((3, 9 + 3))]
    ])
    Da_th = np.block([
        [-x.ierr[:, None], -perr[:, None], -verr[:, None], np.zeros((3, 2))]
    ])

    # double-check the derivatives
    def a_fn(th2):
        th2 = Param.from_arr(th2)
        feedback = (
            - th2.ki * x.ierr
            - th2.kp * (x.p - xd.p_d)
            - th2.kv * (x.v - xd.v_d)
        )
        return feedback + xd.a_d + g
    finitediff_check(th.to_arr(), Da_th, a_fn, Param.dim_str, lambda i: "xyz"[i])

    thrust = np.linalg.norm(a)
    Dthrust_a = (a / thrust).reshape((1, 3))

    zgoal, Dzgoal_a = normalize(a)
    assert np.allclose(zgoal, a / thrust)
    xgoalflat = np.array([np.cos(xd.y_d), np.sin(xd.y_d), 0])

    ygoalnn, Dygoalnn_zgoal, _ = cross(zgoal, xgoalflat)
    assert ygoalnn.shape == (3,)
    ygoal, Dygoal_ygoalnn = normalize(ygoalnn)
    Dygoal_a = Dygoal_ygoalnn @ Dygoalnn_zgoal @ Dzgoal_a

    xgoal, Dxgoal_ygoal, Dxgoal_zgoal = cross(ygoal, zgoal)
    assert xgoal.shape == (3,)
    assert np.isclose(np.linalg.norm(xgoal), 1)
    Dxgoal_a = Dxgoal_ygoal @ Dygoal_a + Dxgoal_zgoal @ Dzgoal_a

    Rd3 = np.stack([xgoal, ygoal, zgoal]).T
    det = np.linalg.det(Rd3)
    RTR = Rd3.T @ Rd3
    assert np.isclose(det, 1)
    assert np.allclose(RTR, np.eye(3))
    DRd3_a = np.block([
        [Dxgoal_a],
        [Dygoal_a],
        [Dzgoal_a],
    ])
    assert DRd3_a.shape == (9, 3)

    # attitude part components
    R3 = x.R.reshape((3, 3)).T
    Z93 = np.zeros((9, 3))
    DR3_x = np.block([Z93, Z93, Z93, np.eye(9), Z93])
    er3, Der3_R3, Der3_Rd3 = SO3error(R3, Rd3)

    #erold, *_ = angleto(zgoal, up)
    #assert np.sign(erold) == np.sign(er3[1])

    # double-check the derivatives
    # def angleto_lambda(xflat):
    #     a, b = xflat.reshape((2, 2))
    #     return angleto(a, b)[0]
    # D = np.concatenate([Der_upgoal, Der_up])[None, :]
    # finitediff_check(np.concatenate([upgoal, up]), D, angleto_lambda, lambda i: "vecs")

    ew = x.w - xd.w_d
    torque = -th.kr * er3 - th.kw * ew
    u = Action(thrust=thrust, torque=torque)

    # controller chain rules
    Dthrust_x = Dthrust_a @ Da_x
    Dthrust_th = Dthrust_a @ Da_th

    #Der_x = Der_up @ Dup_x + Der_upgoal @ Dupgoal_a @ Da_x
    Der3_x = Der3_R3 @ DR3_x + Der3_Rd3 @ DRd3_a @ Da_x

    #Der_th = Der_upgoal @ Dupgoal_a @ Da_th
    Der3_th = Der3_Rd3 @ DRd3_a @ Da_th

    Dtorque_xw = np.zeros((3, State.size))
    Dtorque_xw[:, -3:] = -th.kw * np.eye(3)
    Dtorque_x = -th.kr * Der3_x + Dtorque_xw

    #Dtorque_th = -th.kr * Der_th + np.array([[0, 0, 0, -er, -ew]])
    Z3 = np.zeros(3)
    Dtorque_th = -th.kr * Der3_th + np.stack([Z3, Z3, Z3, -er3, -ew]).T

    Du_x = np.block([
        [Dthrust_x],
        [Dtorque_x],
    ])
    Du_th = np.block([
        [Dthrust_th],
        [Dtorque_th],
    ])
    return u, Du_x, Du_th


def dynamics_py(x: State, xd: Target, u: Action, c: Const):
    """Returns: x, Dx_x, Dx_u."""
    # DYNAMICS
    # --------

    R = x.R.reshape((3, 3)).T
    up = R[:, 2]
    Z33 = np.zeros((3, 3))
    Dup_x = np.block([[Z33, Z33, Z33, Z33, Z33, np.eye(3), Z33]])
    g = np.array([0, 0, c.g])

    # Normally I would use symplectic Euler integration, but plain forward
    # Euler gives simpler Jacobians.

    acc = u.thrust * up - g
    Dacc_x = u.thrust * Dup_x
    #R_t = R @ SO3.exp(SO3.hat(c.dt * x.w))
    # TODO: correct Jacobian for above. For now, using forward Euler, if we
    # want to actually integrate the dynamics we must project R_t onto SO(3)
    # *after* checking derivatives. But I believe it may be too computationally
    # expensive to run on the Crazyflie anyway.
    R_t = R + c.dt * R @ hat(x.w)
    x_t = State(
        ierr = x.ierr + c.dt * (x.p - xd.p_d),
        p = x.p + c.dt * x.v,
        v = x.v + c.dt * acc,
        R = R_t.T.flatten(),
        w = x.w + c.dt * u.torque,
    )
    # TODO: This became trivial after we went from angle state to rotation
    # matrix -- condense some ops.
    Dvt_R = c.dt * Dacc_x[:, 9:-3]
    assert Dvt_R.shape == (3, 9)

    I3 = np.eye(3)
    I9 = np.eye(9)
    Z31 = np.zeros((3, 1))
    Z33 = np.zeros((3, 3))
    Z39 = np.zeros((3, 9))
    Z93 = Z39.T

    DRt_R = np.eye(9) + c.dt * np.kron(hat(-x.w), I3)

    Rx, Ry, Rz = (R.T)[:, :, None]
    DRt_w = c.dt * np.block([
        [Z31, -Rz,  Ry],
        [ Rz, Z31, -Rx],
        [-Ry,  Rx, Z31],
    ])
    assert DRt_w.shape == (9, 3)

    dt3 = c.dt * I3
    Dx_x = np.block([
        [ I3, dt3, Z33,   Z39,   Z33],
        [Z33,  I3, dt3,   Z39,   Z33],
        [Z33, Z33,  I3, Dvt_R,   Z33],
        [Z93, Z93, Z93, DRt_R, DRt_w],
        [Z33, Z33, Z33,   Z39,    I3],
    ])
    # (Refers to Dx_x construction above.) Skipping Coriolis term that would
    # make dw'/dw nonzero because it requires system ID of the inertia matrix,
    # which we can otherwise skip. For the Crazyflie this term can be neglected
    # as the quad's inertia is very small.

    Z91 = np.zeros((9, 1))
    Dx_u = np.block([
        [             Z31, Z33],
        [             Z31, Z33],
        [c.dt * R[:, [2]], Z33],
        [             Z91, Z93],
        [             Z31, dt3],
    ])

    return x_t, Dx_x, Dx_u


def cost_py(x: State, t: Target, u: Action, Q: CostParam):
    """Returns: c, Dc_x, Dc_u."""

    perr = x.p - t.p_d
    verr = x.v - t.v_d
    werr = x.w - t.w_d

    c = 0.5 * (
        Q.p * sqnorm(perr)
        + Q.v * sqnorm(verr)
        + Q.w * sqnorm(werr)
        + Q.thrust * (u.thrust * u.thrust)
        + Q.torque * sqnorm(u.torque)
    )
    Dc_x = np.block([[
        np.zeros((1, 3)),
        Q.p * perr,
        Q.v * verr,
        np.zeros((1, 9)),
        Q.w * werr,
    ]])
    Dc_u = np.block([[
        Q.thrust * u.thrust,
        Q.torque * u.torque,
    ]])

    assert np.allclose(c_bind, c)
    assert np.allclose(Dc_x_bind, Dc_x)
    assert np.allclose(Dc_u_bind, Dc_u)

    return c, Dc_x, Dc_u

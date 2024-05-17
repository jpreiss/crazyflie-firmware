import itertools as it

import numpy as np

import gapsquad
from gapsquad import SO3error, cross, hat, normalize
import SO3

import symforce
import symforce.symbolic as sf
import sympy as sym

from testlib import *


def normalize(x):
    return x / x.norm()


def ctrl_symfn(
    ierr: sf.Vector3, p: sf.Vector3, v: sf.Vector3, R: sf.Matrix33, w: sf.Vector3,
    p_d: sf.Vector3, v_d: sf.Vector3, a_d: sf.Vector3, y_d: sf.Scalar, w_d: sf.Vector3,
    theta_pos: sf.Vector6, theta_rot: sf.Vector4,
    dt: sf.Scalar,
    ):

    ki_xy, ki_z, kp_xy, kp_z, kv_xy, kv_z = theta_pos
    kr_xy, kr_z, kw_xy, kw_z = theta_rot
    ki = sf.Matrix.diag([ki_xy, ki_xy, ki_z])
    kp = sf.Matrix.diag([kp_xy, kp_xy, kp_z])
    kv = sf.Matrix.diag([kv_xy, kv_xy, kv_z])
    kr = 10 * sf.Matrix.diag([kr_xy, kr_xy, kr_z])
    kw = 10 * sf.Matrix.diag([kw_xy, kw_xy, kw_z])

    perr = p - p_d
    verr = v - v_d
    feedback = - ki * ierr - kp * perr - kv * verr
    a = feedback + a_d + sf.Vector3([0, 0, 9.81])

    Rx, Ry, Rz = R.col(0), R.col(1), R.col(2)
    thrust = a.dot(Rz)

    # TODO: handle a \approx 0 case
    zgoal = normalize(a)
    xgoalflat = sf.Vector3([sf.cos(y_d), sf.sin(y_d), 0])
    ygoal = normalize(zgoal.cross(xgoalflat))
    xgoal = ygoal.cross(zgoal)
    Rd = sf.Matrix33.column_stack(xgoal, ygoal, zgoal)

    eRm = 0.5 * (Rd.T * R - R.T * Rd)
    # TODO: switch to log map
    er = sf.Vector3([eRm[2, 1], eRm[0, 2], eRm[1, 0]])
    # TODO: rotate w_d from desired attitude to current attitude?
    ew = w - w_d
    torque = -kr * er - kw * ew

    # TODO: figure out idiomatic way to do this with elementwise tanh, diag
    # matrices, etc.
    RP_LIM = 268
    Y_LIM = 56
    torque = sf.Vector3([
        RP_LIM * sf.tanh(torque[0] / RP_LIM),
        RP_LIM * sf.tanh(torque[1] / RP_LIM),
        Y_LIM * sf.tanh(torque[2] / Y_LIM),
    ])

    return thrust, torque


# Do the codegen in python (TODO: C++)
cg = symforce.codegen.Codegen.function(
    func=ctrl_symfn,
    config=symforce.codegen.PythonConfig(),
)
cg2 = cg.with_jacobians(
    which_args=["ierr", "p", "v", "R", "w", "theta_pos", "theta_rot"],
    which_results=range(2),
    include_results=True,
    name="ctrl",
)
data = cg2.generate_function()
ctrl_codegen = symforce.codegen.codegen_util.load_generated_function(
    "ctrl", data.function_dir)


def ctrl_py(x: State, xd: Target, th: Param, dt: float):
    """Returns: u, Du_x, Du_th."""

    R = x.R.reshape(3, 3).T
    outputs = ctrl_codegen(
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


# Autodiffable fn for dynamics
def dynamics_symfn(
    ierr: sf.Vector3, p: sf.Vector3, v: sf.Vector3, R: sf.Matrix33, w: sf.Vector3,
    p_d: sf.Vector3,
    thrust: sf.Scalar, torque: sf.Vector3,
    dt: sf.Scalar,
    ):
    # position
    up = R[:, 2]
    gvec = sf.Vector3([0, 0, 9.81])
    acc = thrust * up - gvec
    ierrt = ierr + dt * (p - p_d)
    pt = p + dt * v
    vt = v + dt * acc

    # attitude
    expw = sf.Rot3.from_tangent(dt * w).to_rotation_matrix()
    Rt = R * expw
    wt = w + dt * torque

    return ierrt, pt, vt, Rt, wt


# Do the codegen in python (TODO: C++)
cg = symforce.codegen.Codegen.function(
    func=dynamics_symfn,
    config=symforce.codegen.PythonConfig(),
)
cg2 = cg.with_jacobians(
    which_args=["ierr", "p", "v", "R", "w", "thrust", "torque"],
    which_results=range(5),
    include_results=True,
    name="dynamics",
)
data = cg2.generate_function()
dynamics_codegen = symforce.codegen.codegen_util.load_generated_function(
    "dynamics", data.function_dir)


def dynamics_py(x: State, xd: Target, u: Action, dt: float):
    """Returns: x, Dx_x, Dx_u."""

    R = x.R.reshape(3, 3).T
    outputs = dynamics_codegen(
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

    return c, Dc_x, Dc_u

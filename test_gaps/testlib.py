from collections import namedtuple
import sys

from colorama import Fore
import numpy as np

import gapsquad
import SO3


def namedvec(name, fields, sizes):
    """Namedtuple plus helpers for going to/from concatenated arrays."""
    fields = fields.split(" ")
    sizes = sizes.split(" ")
    assert len(fields) == len(sizes)
    sizes = [int(s) for s in sizes]
    splits = np.cumsum(sizes)
    base = namedtuple(name, fields)

    def to_arr(self):
        #for v in [*self]:
            #assert isinstance(v, np.ndarray) or isinstance(v, float)
        return np.concatenate([np.atleast_1d(x) for x in self])

    @classmethod
    def from_arr(cls, arr):
        blocks = map(np.squeeze, np.split(arr, splits[:-1]))
        return cls(*blocks)

    @classmethod
    def dim_str(cls, dim):
        """Converts index into the concatenated vector to subfield index."""
        idx = np.argmin(dim >= splits)
        inner_idx = dim - splits[idx] + sizes[idx]
        return f"{fields[idx]}[{inner_idx}]"

    return type(
        name,
        (base,),
        dict(size=np.sum(sizes), to_arr=to_arr, from_arr=from_arr, dim_str=dim_str),
    )


State = namedvec("State", "ierr p v logR w", "3 3 3 3 3")
Action = namedvec("Action", "thrust torque", "1 3")
Target = namedvec("Target", "p_d v_d a_d w_d", "3 3 3 3")
Param = namedvec("Param", "ki_xy ki_z kp_xy kp_z kv_xy kv_z kr_xy kr_z kw_xy kw_z", "1 1 1 1 1 1 1 1 1 1")
CostParam = namedvec("CostParam", "p v w thrust torque reg_L2", "1 1 1 1 1 1")


def sqnorm(x):
    return np.sum(x ** 2)


def print_with_highlight(x, mask, dim_str):
    """Prints a namedvec"""
    assert len(x.shape) == 1
    n = x.size
    rows = np.empty((2, n), dtype=object)
    for i in range(n):
        if not mask[i]:
            rows[0, i] = str(x[i])
            rows[1, i] = ""
        else:
            rows[0, i] = f"{Fore.RED}{x[i]}{Fore.RESET}"
            name = dim_str(i)
            rows[1, i] = f"{Fore.RED}^ {name}{Fore.RESET}"
    lens = np.vectorize(len)(rows) + 1
    lens = np.amax(lens, axis=0)
    if not np.any(mask):
        rows = rows[[0]]
    for row in rows:
        for s, l in zip(row, lens):
            sys.stdout.write(s.ljust(l))
        print()


def ctrl_cpp(x: State, xd: Target, th: Param, dt: float):
    urets, Jux, Jut = gapsquad.ctrl(x, xd, th, dt)
    u = Action(*urets)
    assert Jux.shape == (Action.size, State.size)
    assert Jut.shape == (Action.size, Param.size)
    return u, Jux, Jut


def dynamics_cpp(x: State, xd: Target, u: Action, dt: float):
    xtrets, Jxx, Jxu = gapsquad.dynamics(x, xd, u, dt)
    xt = State(*xtrets)
    return xt, Jxx, Jxu


def cost_cpp(x: State, t: Target, u: Action, Q: CostParam):
    c_bind, Dc_x_bind, Dc_u_bind = gapsquad.cost(x, t, u, Q)
    assert Dc_x_bind.shape == (State.size,)
    assert Dc_u_bind.shape == (Action.size,)
    # Eigen has no concept of 1D vectors, so pybind11 interprets Nx1 or 1xN
    # matrices as vectors, but in our case we really want a 1xN matrix so we
    # don't need separate code to handle derivatives of scalar-valued vs.
    # vector-valued functions.
    return c_bind, Dc_x_bind[None, :], Dc_u_bind[None, :]


def random_inputs(rng):
    ierr, p, v, logR, w = rng.normal(size=(5, 3))
    x = State(ierr=ierr, p=p, v=v, logR=logR, w=w)

    pd, vd, ad, wd = rng.normal(size=(4, 3))
    yd = rng.normal()
    xd = Target(p_d=pd, v_d=vd, a_d=ad, w_d=wd)

    th = Param.from_arr(np.log(rng.uniform(0.1, 4, size=Param.size)))

    # These CostParams keep the cost output around the same scale as the
    # ctrl and dynamics outputs. If the costs get bigger our finite
    # difference error bounds don't apply anymore.
    Q = CostParam.from_arr(10 ** rng.uniform(-3, -1, size=CostParam.size))

    u = Action.from_arr(rng.random(size=4))

    return x, xd, th, Q, u


def default_inputs(detune=1.0):
    Z3 = np.zeros(3)
    x = State(ierr=Z3, p=Z3, v=Z3, logR=Z3, w=Z3)
    xd = Target(p_d=Z3, v_d=Z3, a_d=Z3, w_d=Z3)
    # initially I tested with simpler params, but once you start testing closed
    # loop, gotta be careful!
    th = Param(
        ki_xy=np.log(1.56),
        ki_z =np.log(1.56),
        kp_xy=np.log(12.5),
        kp_z =np.log(39.0),
        kv_xy=np.log(6.25),
        kv_z =np.log(12.5),
        kr_xy=np.log(1660.0),
        kr_z =np.log(294.0),
        kw_xy=np.log(237.5),
        kw_z =np.log(29.7),
    )
    th = Param.from_arr(th.to_arr() + np.log(detune))
    u = Action(thrust=0, torque=Z3)
    Q = CostParam(p=1, v=0, w=0, thrust=0, torque=0, reg_L2=0)
    return x, xd, th, Q, u

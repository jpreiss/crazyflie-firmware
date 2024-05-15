from collections import namedtuple

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
        for v in [*self]:
            assert isinstance(v, np.ndarray) or isinstance(v, float)
        return np.block([*self])

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


State = namedvec("State", "ierr p v R w", "3 3 3 9 3")
Action = namedvec("Action", "thrust torque", "1 3")
Target = namedvec("Target", "p_d v_d a_d y_d w_d", "3 3 3 1 3")
Param = namedvec("Param", "ki_xy ki_z kp_xy kp_z kv_xy kv_z kr_xy kr_z kw_xy kw_z", "1 1 1 1 1 1 1 1 1 1")
CostParam = namedvec("CostParam", "p v w thrust torque reg_L2", "1 1 1 1 1 1")
Const = namedvec("Const", "g m j dt", "1 1 3 1")


def sqnorm(x):
    return np.sum(x ** 2)


# utilities since our Eigen fns take 3x3 matrix, not flat 9x1.
def state2args(x: State):
    xargs = [*x]
    for i in range(len(xargs)):
        if xargs[i].shape == (9,):
            xargs[i] = xargs[i].reshape(3, 3).T
    return xargs

def rets2state(xrets):
    xrets = list(xrets)
    for i in range(len(xrets)):
        if xrets[i].shape == (3, 3):
            assert xrets[i].flags["C"]
            assert not xrets[i].flags["F"]
            xrets[i] = xrets[i].T.reshape(9)
    return State(*xrets)


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


def ctrl_cpp(x: State, xd: Target, th: Param, c: Const):
    xargs = state2args(x)
    urets, Jux, Jut = gapsquad.ctrl(xargs, xd, th, c.dt)
    u = Action(*urets)
    assert Jux.shape == (Action.size, State.size)
    assert Jut.shape == (Action.size, Param.size)
    return u, Jux, Jut


def dynamics_cpp(x: State, xd: Target, u: Action, c: Const):
    xargs = state2args(x)
    xrets, Jxx, Jxu = gapsquad.dynamics(xargs, xd, u, c.dt)
    xt = rets2state(xrets)
    return xt, Jxx, Jxu


def cost_cpp(x: State, t: Target, u: Action, Q: CostParam):
    xargs = state2args(x)
    c_bind, Dc_x_bind, Dc_u_bind = gapsquad.cost(xargs, t, u, Q)
    assert Dc_x_bind.shape == (State.size,)
    assert Dc_u_bind.shape == (Action.size,)
    # Eigen has no concept of 1D vectors, so pybind11 interprets Nx1 or 1xN
    # matrices as vectors, but in our case we really want a 1xN matrix so we
    # don't need separate code to handle derivatives of scalar-valued vs.
    # vector-valued functions.
    return c_bind, Dc_x_bind[None, :], Dc_u_bind[None, :]


def random_inputs(rng):
    ierr, p, v, w = rng.normal(size=(4, 3))
    R = SO3.random(rng, 3)
    assert np.allclose(np.eye(3), R.T @ R)
    assert np.isclose(np.linalg.det(R), 1)
    x = State(ierr=ierr, p=p, v=v, R=R.flatten(), w=w)

    pd, vd, ad, wd = rng.normal(size=(4, 3))
    yd = rng.normal()
    xd = Target(p_d=pd, v_d=vd, a_d=ad, y_d=yd, w_d=wd)

    th = Param.from_arr(rng.uniform(0.1, 4, size=Param.size))

    # These CostParams keep the cost output around the same scale as the
    # ctrl and dynamics outputs. If the costs get bigger our finite
    # difference error bounds don't apply anymore.
    cp = CostParam.from_arr(10 ** rng.uniform(-3, -1, size=CostParam.size))

    return x, xd, th, cp

import numpy as np
import pytest

from testlib import *
from purepy import *


dt = 0.01


def _to_arr(x):
    if any(isinstance(x, C) for C in [np.ndarray, np.float64, float]):
        return x
    return x.to_arr()


def test_py_cpp_ctrl():
    rng = np.random.default_rng(0)
    fns = [ctrl_cpp, ctrl_py]
    for i in range(100):
        x, xd, th, *_ = random_inputs(rng)
        outputs = [fn(x, xd, th, dt) for fn in fns]
        for y1, y2 in zip(*outputs):
            y1, y2 = map(_to_arr, [y1, y2])
            assert np.allclose(y1, y2)


def test_py_cpp_dynamics():
    rng = np.random.default_rng(0)
    fns = [dynamics_cpp, dynamics_py]
    for i in range(100):
        x, xd, *_ = random_inputs(rng)
        u = Action.from_arr(rng.normal(size=4))
        outputs = [fn(x, xd, u, dt) for fn in fns]
        for y1, y2 in zip(*outputs):
            y1, y2 = map(_to_arr, [y1, y2])
            # TODO: implement true exp(K) in C++ so we can tighten this again.
            assert np.allclose(y1, y2, rtol=1e-4, atol=1e-5)


def test_py_cpp_cost():
    rng = np.random.default_rng(0)
    fns = [cost_cpp, cost_py]
    for i in range(100):
        x, xd, _, cp, _ = random_inputs(rng)
        u = Action.from_arr(rng.normal(size=4))
        outputs = [fn(x, xd, u, cp) for fn in fns]
        for y1, y2 in zip(*outputs):
            y1, y2 = map(_to_arr, [y1, y2])
            assert np.allclose(y1, y2)

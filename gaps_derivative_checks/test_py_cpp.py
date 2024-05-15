from testlib import *
from purepy import *


const = Const(g=9.81, m=1, j=None, dt=0.1)


def test_py_cpp_ctrl():
    rng = np.random.default_rng(0)
    fns = [ctrl_cpp, ctrl_py]
    for i in range(100):
        x, xd, th, _ = random_inputs(rng)
        outputs = [fn(x, xd, th, const) for fn in fns]
        for y1, y2 in zip(*outputs):
            assert np.allclose(y1, y2)


def test_py_cpp_dynamics():
    rng = np.random.default_rng(0)
    fns = [dynamics_cpp, dynamics_py]
    for i in range(100):
        x, xd, *_ = random_inputs(rng)
        u = Action.from_arr(rng.normal(size=4))
        outputs = [fn(x, xd, u, const) for fn in fns]
        for y1, y2 in zip(*outputs):
            assert np.allclose(y1, y2)


def test_py_cpp_cost():
    rng = np.random.default_rng(0)
    fns = [cost_cpp, cost_py]
    for i in range(100):
        x, xd, _, cp = random_inputs(rng)
        u = Action.from_arr(rng.normal(size=4))
        outputs = [fn(x, xd, u, cp) for fn in fns]
        for y1, y2 in zip(*outputs):
            assert np.allclose(y1, y2)

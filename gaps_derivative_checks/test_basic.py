import numpy as np

from testlib import *


def test_cost_sanity():
    Z3 = np.zeros(3)
    I = np.eye(3)
    x = State(ierr=Z3, p=Z3, v=Z3, R=I.flatten(), w=Z3)
    t = Target(p_d=Z3, v_d=Z3, a_d=Z3, y_d=0, w_d=Z3)
    u = Action(thrust=0, torque=Z3)
    Q = CostParam(p=1, v=1, w=1, thrust=1, torque=1, reg_L2=1)
    c, Dc_x, Dc_u = cost_cpp(x, t, u, Q)
    assert np.isclose(c, 0)

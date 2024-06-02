# -----------------------------------------------------------------------------
# This file was autogenerated by symforce from template:
#     function/FUNCTION.py.jinja
# Do NOT modify by hand.
# -----------------------------------------------------------------------------

# pylint: disable=too-many-locals,too-many-lines,too-many-statements,unused-argument,unused-import

import math
import typing as T

import numpy

import sym


def cost(p, v, w, p_d, v_d, w_d, thrust, torque, Qp, Qv, Qw, Qthrust, Qtorque):
    # type: (numpy.ndarray, numpy.ndarray, numpy.ndarray, numpy.ndarray, numpy.ndarray, numpy.ndarray, float, numpy.ndarray, float, float, float, float, float) -> T.Tuple[float, numpy.ndarray]
    """
    This function was autogenerated from a symbolic function. Do not modify by hand.

    Symbolic function: cost_symfn

    Args:
        p: Matrix31
        v: Matrix31
        w: Matrix31
        p_d: Matrix31
        v_d: Matrix31
        w_d: Matrix31
        thrust: Scalar
        torque: Matrix31
        Qp: Scalar
        Qv: Scalar
        Qw: Scalar
        Qthrust: Scalar
        Qtorque: Scalar

    Outputs:
        cost: Scalar
        jacobian: (1x13) jacobian of cost wrt args p (3), v (3), w (3), thrust (1), torque (3)
    """

    # Total ops: 62

    # Input arrays
    if p.shape == (3,):
        p = p.reshape((3, 1))
    elif p.shape != (3, 1):
        raise IndexError(
            "p is expected to have shape (3, 1) or (3,); instead had shape {}".format(p.shape)
        )

    if v.shape == (3,):
        v = v.reshape((3, 1))
    elif v.shape != (3, 1):
        raise IndexError(
            "v is expected to have shape (3, 1) or (3,); instead had shape {}".format(v.shape)
        )

    if w.shape == (3,):
        w = w.reshape((3, 1))
    elif w.shape != (3, 1):
        raise IndexError(
            "w is expected to have shape (3, 1) or (3,); instead had shape {}".format(w.shape)
        )

    if p_d.shape == (3,):
        p_d = p_d.reshape((3, 1))
    elif p_d.shape != (3, 1):
        raise IndexError(
            "p_d is expected to have shape (3, 1) or (3,); instead had shape {}".format(p_d.shape)
        )

    if v_d.shape == (3,):
        v_d = v_d.reshape((3, 1))
    elif v_d.shape != (3, 1):
        raise IndexError(
            "v_d is expected to have shape (3, 1) or (3,); instead had shape {}".format(v_d.shape)
        )

    if w_d.shape == (3,):
        w_d = w_d.reshape((3, 1))
    elif w_d.shape != (3, 1):
        raise IndexError(
            "w_d is expected to have shape (3, 1) or (3,); instead had shape {}".format(w_d.shape)
        )

    if torque.shape == (3,):
        torque = torque.reshape((3, 1))
    elif torque.shape != (3, 1):
        raise IndexError(
            "torque is expected to have shape (3, 1) or (3,); instead had shape {}".format(
                torque.shape
            )
        )

    # Intermediate terms (14)
    _tmp0 = w[1, 0] - w_d[1, 0]
    _tmp1 = w[0, 0] - w_d[0, 0]
    _tmp2 = w[2, 0] - w_d[2, 0]
    _tmp3 = p[2, 0] - p_d[2, 0]
    _tmp4 = p[0, 0] - p_d[0, 0]
    _tmp5 = p[1, 0] - p_d[1, 0]
    _tmp6 = v[1, 0] - v_d[1, 0]
    _tmp7 = v[2, 0] - v_d[2, 0]
    _tmp8 = v[0, 0] - v_d[0, 0]
    _tmp9 = 1.00000000000000
    _tmp10 = Qp * _tmp9
    _tmp11 = Qv * _tmp9
    _tmp12 = Qw * _tmp9
    _tmp13 = Qtorque * _tmp9

    # Output terms
    _cost = (
        0.5 * Qp * (_tmp3**2 + _tmp4**2 + _tmp5**2)
        + 0.5 * Qthrust * thrust**2
        + 0.5 * Qtorque * (torque[0, 0] ** 2 + torque[1, 0] ** 2 + torque[2, 0] ** 2)
        + 0.5 * Qv * (_tmp6**2 + _tmp7**2 + _tmp8**2)
        + 0.5 * Qw * (_tmp0**2 + _tmp1**2 + _tmp2**2)
    )
    _jacobian = numpy.zeros(13)
    _jacobian[0] = _tmp10 * _tmp4
    _jacobian[1] = _tmp10 * _tmp5
    _jacobian[2] = _tmp10 * _tmp3
    _jacobian[3] = _tmp11 * _tmp8
    _jacobian[4] = _tmp11 * _tmp6
    _jacobian[5] = _tmp11 * _tmp7
    _jacobian[6] = _tmp1 * _tmp12
    _jacobian[7] = _tmp0 * _tmp12
    _jacobian[8] = _tmp12 * _tmp2
    _jacobian[9] = Qthrust * _tmp9 * thrust
    _jacobian[10] = _tmp13 * torque[0, 0]
    _jacobian[11] = _tmp13 * torque[1, 0]
    _jacobian[12] = _tmp13 * torque[2, 0]
    return _cost, _jacobian

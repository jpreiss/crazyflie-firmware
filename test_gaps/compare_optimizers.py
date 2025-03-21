import sys

import matplotlib.pyplot as plt
import numpy as np
import symforce.symbolic as sf


sys.path.append("../..")
from crazyswarm.ros_ws.src.crazyswarm.scripts.uav_trajectory import compute_omega, TrigTrajectory

from testlib import *

PERIOD = 4
RADIUS = 0.75
Q = CostParam(
    p=1e0,
    v=1e-4,
    w=1e-3,
    thrust=1e-8,
    torque=1e-7,
    reg_L2=1e-10,
)

def logR_from_acc(acc):
    thrust = sf.Vector3(acc) + sf.Vector3([0, 0, 9.81])
    zgoal = thrust / thrust.norm()
    e_z = sf.Vector3((0, 0, 1))
    Rgoal = sf.Rot3.from_two_unit_vectors(e_z, zgoal)
    return np.array(Rgoal.to_tangent()).astype(float)

# Since codegen includes exponentiation, we need to undo it for non-transformed methods.
# Let phi = log(theta). GAPS gives dF / dphi. By chain rule,
# dF / dtheta = dF / dphi * dphi / dtheta
#             = dF / dphi * 1 / theta
#             = dF / dphi * 1 / exp(phi).

def step_ogd(phi, grad, rate):
    theta = np.exp(phi)
    grad2 = grad / theta
    theta -= rate * grad2
    return np.log(theta)

def step_omd(phi, grad, rate):
    theta = np.exp(phi)
    grad2 = grad / theta
    # TODO: mirror map
    theta -= rate * grad2
    return np.log(theta)

def step_reparam(phi, grad, rate):
    return phi - rate * grad


def run(opt, rate):
    traj_major = TrigTrajectory.Cosine(amplitude=RADIUS, period=PERIOD)
    traj_minor = TrigTrajectory.Sine(amplitude=RADIUS/2, period=PERIOD/2)

    dt = 1 / 500
    laps = 8
    T = int(laps * PERIOD / dt)

    derivs = np.zeros((4, 3))
    yaw = 0
    angvel = np.zeros(3)

    phi = HI_GAIN_THETA - np.log(2)
    if opt is not None:
        phi -= np.log(2)
    y = np.zeros((State.size, Param.size))

    costs = np.zeros(T)

    for t in range(T):
        major = traj_major(dt * t)
        minor = traj_minor(dt * t)

        derivs[:, 0] = major
        # diagonal traj
        derivs[:, 1] = -minor
        derivs[:, 2] = minor

        pos, vel, acc, jerk = derivs
        angvel = compute_omega(acc, jerk, yaw=0, dyaw=0)
        target = Target(pos, vel, acc, angvel)

        if t == 0:
            logR = logR_from_acc(acc)
            x = State(ierr=np.zeros(3), p=pos, v=vel, logR=logR, w=angvel)

        u, Du_x, Du_t = ctrl_cpp(x, target, Param.from_arr(phi), dt)
        x, Dx_x, Dx_u = dynamics_cpp(x, target, u, dt)
        c, Dc_x, Dc_u = cost_cpp(x, target, u, Q)
        costs[t] = c

        # extra mass disturbance
        vnew = x.v - dt * np.array([0, 0, 1.0])
        x = x._replace(v=vnew)

        # GAPS
        grad = (Dc_x + Dc_u @ Du_x) @ y + Dc_u @ Du_t
        grad += Q.reg_L2 * phi
        grad = grad.squeeze()
        y = (Dx_x + Dx_u @ Du_x) @ y + Dx_u @ Du_t

        # update
        if opt is not None:
            phi = opt(phi, grad, rate)

    return costs


# for some reason importing gapsquad doesn't work right unless this runs under
# pytest... didn't feel like figuring it out
def test_main():
    cost_base = run(None, None)
    costs = run(step_reparam, 2e-2)
    plt.plot(np.cumsum(costs - cost_base))
    plt.savefig("gaps_grads.pdf")

if __name__ == "__main__":
    main()

import itertools as it
import multiprocessing
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import symforce.symbolic as sf


sys.path.append("../..")
from crazyswarm.ros_ws.src.crazyswarm.scripts.uav_trajectory import compute_omega, TrigTrajectory

from testlib import *

# experiment constants
PERIOD = 4
RADIUS = 0.75
LAPS = 4
Q = CostParam(
    p=1e0,
    v=1e-4,
    w=1e-3,
    thrust=1e-8,
    torque=1e-7,
    reg_L2=1e-10,
)

# plotting constants
RATE = "learning rate $\\eta$"
REGRET = '"regret" vs. expert'

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

def OMD(phi, grad, rate, mgrad, mgradinv):
    theta = np.exp(phi)
    grad2 = grad / theta
    theta = mgradinv(mgrad(theta) - rate * grad2)
    return np.log(theta)

eucsq_grad = eucsq_gradinv = lambda x : x
ent_grad = lambda x : np.log(x) - 1
ent_gradinv = lambda x : np.exp(x + 1)
log_grad = log_gradinv = lambda x : -1 / x

def grad(phi, grad, rate):
    return OMD(phi, grad, rate, eucsq_grad, eucsq_gradinv)

def mirror_KL(phi, grad, rate):
    return OMD(phi, grad, rate, ent_grad, ent_gradinv)

# expand:
# phi' = log(theta')
#      = log(log_gradinv(log_grad(theta) - rate * grad / theta))
#      = log(log_gradinv(log_grad(exp(phi)) - rate * grad / exp(phi)))
#      = log(-1/(-1/exp(phi) - rate * grad / exp(phi)))
#      = log(1/(1/exp(phi) + rate * grad / exp(phi)))
#      = -log(1/exp(phi) + rate * grad / exp(phi))
#      = -log(1 + rate * grad) - log(1/exp(phi))
#      = -log(1 + rate * grad) + phi
# approx phi - rate * grad when rate * grad is small

def OMD_IS(phi, grad, rate):
    return OMD(phi, grad, rate, log_grad, log_gradinv)

def log_param(phi, grad, rate):
    return phi - rate * grad

# These were for debugging to make sure my OMD was correct
# def OGD(phi, grad, rate):
#     theta = np.exp(phi)
#     grad2 = grad / theta
#     theta -= rate * grad2
#     return np.log(theta)

# def MultWeights(phi, grad, rate):
#     theta = np.exp(phi)
#     grad2 = grad / theta
#     theta *= np.exp(-rate * grad2)
#     return np.log(theta)


def run(opt, rate):
    traj_major = TrigTrajectory.Cosine(amplitude=RADIUS, period=PERIOD)
    traj_minor = TrigTrajectory.Sine(amplitude=RADIUS/2, period=PERIOD/2)

    dt = 1 / 500
    T = int(LAPS * PERIOD / dt)

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
    opts = [grad, mirror_KL, log_param]
    rate_lims = [1e-3, 1e1]
    rates = np.geomspace(*rate_lims, 30)
    args = list(it.product(opts, rates))
    pool = multiprocessing.Pool(multiprocessing.cpu_count() - 1)
    costs = pool.starmap(run, args)
    records = []
    for costs, (opt, rate) in zip(costs, args):
        regret = np.sum(costs - cost_base)
        records.append({
            "gradstep": opt.__name__,
            RATE: rate,
            REGRET: regret,
        })
    df = pd.DataFrame(records)
    sns.set_style("whitegrid")
    if True:
        plt.rcParams.update({"text.usetex": True, "font.size": 12})
    grid = sns.relplot(
        kind="line",
        data=df,
        x=RATE,
        y=REGRET,
        hue="gradstep",
        #style="opt",
        aspect=1.4,
        height=2.5,
    )
    grid.set(xlim=rate_lims, xscale="log")
    grid.savefig("gaps_grads.pdf")


if __name__ == "__main__":
    main()

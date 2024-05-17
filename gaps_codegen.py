import symforce
import symforce.symbolic as sf


OUTDIRS = ["./test_gaps/codegen", "./src/modules/interface/codegen"]
CONFIGS = [symforce.codegen.PythonConfig(), symforce.codegen.CppConfig()]


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

    return sf.Matrix.block_matrix([[sf.Vector1(thrust)], [torque]])


for outdir, config in zip(OUTDIRS, CONFIGS):
    cg = symforce.codegen.Codegen.function(
        config=config,
        func=ctrl_symfn,
        output_names=["thrust_torque"],
    )
    cg2 = cg.with_linearization(
        which_args=["ierr", "p", "v", "R", "w", "theta_pos", "theta_rot"],
        linearization_mode=symforce.codegen.LinearizationMode.STACKED_JACOBIAN,
        include_result=True,
        name="ctrl",
    )
    data = cg2.generate_function(output_dir=outdir, skip_directory_nesting=True)


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

    return sf.Matrix.block_matrix([
        [ierrt], [pt], [vt], [Rt.col(0)], [Rt.col(1)], [Rt.col(2)], [wt]
    ])


for outdir, config in zip(OUTDIRS, CONFIGS):
    cg = symforce.codegen.Codegen.function(
        config=config,
        func=dynamics_symfn,
        output_names=["ierr_p_v_R_w"],
    )
    cg2 = cg.with_linearization(
        which_args=["ierr", "p", "v", "R", "w", "thrust", "torque"],
        linearization_mode=symforce.codegen.LinearizationMode.STACKED_JACOBIAN,
        include_result=True,
        name="dynamics",
    )
    data = cg2.generate_function(output_dir=outdir, skip_directory_nesting=True)


def cost_symfn(
    p: sf.Vector3, v: sf.Vector3, w: sf.Vector3,
    p_d: sf.Vector3, v_d: sf.Vector3, w_d: sf.Vector3,
    thrust: sf.Scalar, torque: sf.Vector3,
    Qp: sf.Scalar, Qv: sf.Scalar, Qw: sf.Scalar,
    Qthrust: sf.Scalar, Qtorque: sf.Scalar,
    ):

    c = 0.5 * (
        Qp * (p - p_d).squared_norm()
        + Qv * (v - v_d).squared_norm()
        + Qw * (w - w_d).squared_norm()
        + Qthrust * thrust ** 2
        + Qtorque * torque.squared_norm()
    )
    return c


for outdir, config in zip(OUTDIRS, CONFIGS):
    cg = symforce.codegen.Codegen.function(
        config=config,
        func=cost_symfn,
        output_names=["cost"],
    )
    cg2 = cg.with_linearization(
        which_args=["p", "v", "w", "thrust", "torque"],
        linearization_mode=symforce.codegen.LinearizationMode.STACKED_JACOBIAN,
        include_result=True,
        name="cost",
    )
    data = cg2.generate_function(output_dir=outdir, skip_directory_nesting=True)


# symforce always clobbers it with the most recent function *only*
with open(OUTDIRS[0] + "/__init__.py", "w") as f:
    f.write("""
from .cost import cost
from .ctrl import ctrl
from .dynamics import dynamics
""")


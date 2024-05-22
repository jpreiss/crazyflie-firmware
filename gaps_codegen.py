import symforce
symforce.set_epsilon_to_invalid()
import symforce.symbolic as sf

EPS = 1e-6

def normalize(x):
    return x / x.norm(epsilon=EPS)


def softclip(x, absmax):
    return absmax * sf.tanh(x / absmax)


def ctrl_symfn(
    ierr: sf.Vector3, p: sf.Vector3, v: sf.Vector3, logR: sf.Vector3, w: sf.Vector3,
    p_d: sf.Vector3, v_d: sf.Vector3, a_d: sf.Vector3, y_d: sf.Scalar, w_d: sf.Vector3,
    theta_pos: sf.Vector6, theta_rot: sf.Vector4,
    dt: sf.Scalar,
    ):

    ki_xy, ki_z, kp_xy, kp_z, kv_xy, kv_z = map(sf.exp, theta_pos)
    kr_xy, kr_z, kw_xy, kw_z = map(sf.exp, theta_rot)
    ki = sf.Matrix.diag([ki_xy, ki_xy, ki_z])
    kp = sf.Matrix.diag([kp_xy, kp_xy, kp_z])
    kv = sf.Matrix.diag([kv_xy, kv_xy, kv_z])
    kr = sf.Matrix.diag([kr_xy, kr_xy, kr_z])
    kw = sf.Matrix.diag([kw_xy, kw_xy, kw_z])

    perr = p - p_d
    verr = v - v_d
    feedback = - ki * ierr - kp * perr - kv * verr
    a = feedback + a_d + sf.Vector3([0, 0, 9.81])

    Rz = sf.Rot3.from_tangent(logR, epsilon=EPS) * sf.Vector3((0, 0, 1))
    thrust = a.dot(Rz)

    # TODO: handle a \approx 0 case ?
    zgoal = normalize(a)
    e_z = sf.Vector3((0, 0, 1))
    Rgoal = sf.Rot3.from_two_unit_vectors(e_z, zgoal, epsilon=EPS)
    R = sf.Rot3.from_tangent(logR, epsilon=EPS)
    er = (R * Rgoal.inverse()).to_tangent(epsilon=EPS)
    er = sf.Vector3(er)

    # TODO: rotate w_d from desired attitude to current attitude?
    ew = w - w_d
    torque = -kr * er - kw * ew

    # TODO: figure out idiomatic way to do this with elementwise tanh, diag
    # matrices, etc.
    RP_LIM = 268
    Y_LIM = 56
    torque = sf.Vector3([
        softclip(torque[0], RP_LIM),
        softclip(torque[1], RP_LIM),
        softclip(torque[2], Y_LIM),
    ])

    return sf.Matrix.block_matrix([[sf.Vector1(thrust)], [torque]])


# Autodiffable fn for dynamics
def dynamics_symfn(
    ierr: sf.Vector3, p: sf.Vector3, v: sf.Vector3, logR: sf.Vector3, w: sf.Vector3,
    p_d: sf.Vector3,
    thrust: sf.Scalar, torque: sf.Vector3,
    dt: sf.Scalar,
    ):
    # position
    up = sf.Rot3.from_tangent(logR, epsilon=EPS) * sf.Vector3((0, 0, 1))
    gvec = sf.Vector3([0, 0, 9.81])
    acc = thrust * up - gvec
    ierrt = ierr + dt * (p - p_d)
    pt = p + dt * v
    vt = v + dt * acc

    # attitude
    R = sf.Rot3.from_tangent(logR, epsilon=EPS)
    Rt = R * sf.Rot3.from_tangent(dt * w, epsilon=EPS)
    logRt = sf.Vector3(Rt.to_tangent(epsilon=EPS))
    wt = w + dt * torque

    return sf.Matrix.block_matrix([
        [ierrt], [pt], [vt], [logRt], [wt]
    ])


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


def main():
    OUTDIRS = ["./test_gaps/codegen", "./src/modules/interface/codegen"]
    CONFIGS = [
        symforce.codegen.PythonConfig(),
        symforce.codegen.CppConfig(),
    ]
    # Can comment out on Linux. Was causing problems on Mac.
    CONFIGS[1].render_template_config.autoformat = False

    for outdir, config in zip(OUTDIRS, CONFIGS):
        cg = symforce.codegen.Codegen.function(
            config=config,
            func=ctrl_symfn,
            output_names=["thrust_torque"],
        )
        cg2 = cg.with_linearization(
            which_args=["ierr", "p", "v", "logR", "w", "theta_pos", "theta_rot"],
            linearization_mode=symforce.codegen.LinearizationMode.STACKED_JACOBIAN,
            include_result=True,
            name="ctrl",
        )
        data = cg2.generate_function(output_dir=outdir, skip_directory_nesting=True)


    for outdir, config in zip(OUTDIRS, CONFIGS):
        cg = symforce.codegen.Codegen.function(
            config=config,
            func=dynamics_symfn,
            output_names=["ierr_p_v_logR_w"],
        )
        cg2 = cg.with_linearization(
            which_args=["ierr", "p", "v", "logR", "w", "thrust", "torque"],
            linearization_mode=symforce.codegen.LinearizationMode.STACKED_JACOBIAN,
            include_result=True,
            name="dynamics",
        )
        data = cg2.generate_function(output_dir=outdir, skip_directory_nesting=True)


    # TODO: set epsilon to invalid. cost should not have singularities
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


if __name__ == "__main__":
    main()

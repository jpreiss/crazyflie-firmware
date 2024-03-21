#include <Eigen/Dense>

#include "gaps.h"

namespace {

// inputs
// state is (pos err, vel err, int pos err)
Eigen::Matrix<float, 9, 9> dxdx;
Eigen::Matrix<float, 9, 3> dxdu;
Eigen::Matrix<float, 3, 6> dudtheta;
Eigen::Matrix<float, 3, 9> dudx;
Eigen::Matrix<float, 1, 9> dcdx;
Eigen::Matrix<float, 1, 3> dcdu;

extern "C" void gaps_init(float dt)
{
    // constants
    auto I = Eigen::Matrix<float, 3, 3>::Identity();
    dxdx.setIdentity();
    dxdx.block<3, 3>(0, 3) = dt * I;
    dxdx.block<3, 3>(6, 0) = dt * I;
    dxdu.setZero();
    dxdu.block<3, 3>(3, 0) = -dt * I;
    dcdx.setZero();
}

extern "C" void gaps_reset(float kp_xy, float kp_z, float kd_xy, float kd_z, struct gaps *gaps)
{
    Eigen::Map<Eigen::Matrix<float, 9, 6> > my(gaps->y[0]);
    my.setZero();
}

using Map3 = Eigen::Map<Eigen::Matrix<float, 3, 1> const>;

extern "C" void gaps_update(
    float const pos_err[3],
    float const vel_err[3],
    float const int_pos_err[3],
    float const p_cost,
    float const v_cost,
    float const u_cost,
    float const eta,
    struct gaps *gaps, // inout
    float u[3] // out
    )
{
    // compute u
    float kp_xy = gaps->theta[0];
    float kp_z = gaps->theta[1];
    float kd_xy = gaps->theta[2];
    float kd_z = gaps->theta[3];
    float ki_xy = gaps->theta[4];
    float ki_z = gaps->theta[5];

    u[0] = kp_xy * pos_err[0] + kd_xy * vel_err[0] + ki_xy * int_pos_err[0];
    u[1] = kp_xy * pos_err[1] + kd_xy * vel_err[1] + ki_xy * int_pos_err[1];
    u[2] = kp_z * pos_err[2] + kd_z * vel_err[2] + ki_z * int_pos_err[2];

    // cost derivatives
    dcdx.block<1, 3>(0, 0) = p_cost * Map3(pos_err);
    dcdx.block<1, 3>(0, 3) = v_cost * Map3(vel_err);
    dcdu = u_cost * Map3(u);

    // action derivative wrt theta.
    // kp_xy, kp_z, kd_xy, kd_z, ki_xy, ki_z
    dudtheta <<
        pos_err[0],          0, vel_err[0],          0, int_pos_err[0],              0,  // ux
        pos_err[1],          0, vel_err[1],          0, int_pos_err[1],              0,  // uy
                 0, pos_err[2],          0, vel_err[2],              0, int_pos_err[2];  // uz

    // action derivative wrt x.
    // pos, vel, int_pos
    dudx <<
        kp_xy,     0,    0, kd_xy,     0,    0, ki_xy,     0,    0,  // ux
            0, kp_xy,    0,     0, kd_xy,    0,     0, ki_xy,    0,  // uy
            0,     0, kp_z,     0,     0, kd_z,     0,     0, ki_z;  // uz

    // dxdx, dxdu were constant, set in init.

    // gradient step
    Eigen::Map<Eigen::Matrix<float, 6, 1> > mtheta(gaps->theta);
    Eigen::Map<Eigen::Matrix<float, 9, 6> > my(gaps->y[0]);
    mtheta -= eta * (dcdx * my + dcdu * dudtheta);

    // projection
    for (int i = 0; i < 6; ++i) {
        mtheta[i] = fmaxf(mtheta[i], 0.0f);
    }

    // dynamic programming
    my = (dxdx + dxdu * dudx) * my + dxdu * dudtheta;
}

} // anonymous namespace

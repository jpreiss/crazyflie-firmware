#include <Eigen/Dense>

#include "gaps.h"

namespace {

int const dimX = 6;
int const dimU = 3;
int const dimT = 4;

// state
Eigen::Matrix<float, dimT, 1> theta;
Eigen::Matrix<float, dimX, dimT> y;

// inputs
Eigen::Matrix<float, dimX, dimX> dxdx;
Eigen::Matrix<float, dimX, dimU> dxdu;
Eigen::Matrix<float, dimU, dimT> dudtheta;
Eigen::Matrix<float, 1, dimX> dcdx;
Eigen::Matrix<float, 1, dimU> dcdu;

extern "C" void gaps_init(
    float kp_xy, float kp_z, float kd_xy, float kd_z, float dt)
{
    // state
    theta[0] = kp_xy;
    theta[1] = kp_z;
    theta[2] = kd_xy;
    theta[3] = kd_z;
    y.setZero();

    // constants
    auto I = Eigen::Matrix<float, 3, 3>::Identity();
    dxdx.setIdentity();
    dxdx.block<3, 3>(3, 3) = dt * I;
    dxdu.setZero();
    dxdu.block<3, 3>(3, 0) = dt * I;
}

using Map3 = Eigen::Map<Eigen::Matrix<float, 3, 1> const>;

extern "C" void gaps_update(
    float const pos_err[3],
    float const vel_err[3],
    float const p_cost,
    float const v_cost,
    float const u_cost,
    float const eta,
    float u[3] //out
    )
{
    // compute u
    float kp_xy = theta[0];
    float kp_z = theta[1];
    float kd_xy = theta[2];
    float kd_z = theta[3];

    u[0] = kp_xy * pos_err[0] + kd_xy * vel_err[0];
    u[1] = kp_xy * pos_err[1] + kd_xy * vel_err[1];
    u[2] = kp_z * pos_err[2] + kd_z * vel_err[2];

    // cost derivatives
    dcdx.block<1, 3>(0, 0) = p_cost * Map3(pos_err);
    dcdx.block<1, 3>(0, 3) = v_cost * Map3(vel_err);
    dcdu = u_cost * Map3(u);

    // action derivative wrt theta.
    // kp_xy, kp_z, kd_xy, kd_z
    dudtheta <<
        -pos_err[0],           0, -vel_err[0],           0,  // ux
        -pos_err[1],           0, -vel_err[1],           0,  // uy
                  0, -pos_err[2],           0, -vel_err[2];  // uz

    // dxdx, dxdu were constant, set in init.

    // gradient step
    theta -= eta * (dcdx * y + dcdu * dudtheta);
    // dynamic programming
    y = dxdx * y + dxdu * dudtheta;
}

} // anonymous namespace

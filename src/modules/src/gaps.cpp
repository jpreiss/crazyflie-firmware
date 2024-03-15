#include <Eigen/Dense>

namespace {

int constexpr dimX = 6;
int constexpr dimU = 3;
int constexpr dimT = 2;

// state
Eigen::Matrix<float, dimT, 1> theta;
Eigen::Matrix<float, dimX, dimT> y;

// inputs
Eigen::Matrix<float, dimX, dimX> dxdx;
Eigen::Matrix<float, dimX, dimU> dxdu;
Eigen::Matrix<float, dimU, dimT> dudtheta;
Eigen::Matrix<float, 1, dimX> dcdx;
Eigen::Matrix<float, 1, dimU> dcdu;

void gaps_init(float dt)
{
    auto I = Eigen::Matrix<float, 3, 3>::Identity();
    dxdx.setIdentity();
    dxdx.block<3, 3>(3, 3) = dt * I;
    dxdu.setZero();
    dxdu.block<3, 3>(3, 0) = dt * I;
    // TODO: dudtheta
}

void gaps_fill(float pos_err[3], float vel_err[3], float p_cost, float v_cost, float u_cost)
{
    dcdx.block<1, 3>(0, 0) = p_cost * pos_err;
    dcdx.block<1, 3>(0, 3) = v_cost * vel_err;
    // TODO: u and dcdu
}

// updates the global variables using the inputs
void gaps_update(float eta)
{
    // gradient step
    theta -= eta * (dcdx * y + dcdu * dudtheta);
    // dynamic programming
    y = dxdx * y + dxdu * dudtheta;
}

} // anonymous namespace

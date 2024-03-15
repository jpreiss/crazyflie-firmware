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

// updates the global variables using the inputs
void gaps_update(float eta)
{
    // gradient step
    theta -= eta * (dcdx * y + dcdu * dudtheta);
    // dynamic programming
    y = dxdx * y + dxdu * dudtheta;
}

} // anonymous namespace

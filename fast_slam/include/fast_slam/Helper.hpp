#ifndef FASTSLAM_HELPER_H
#define FASTSLAM_HELPER_H

#include <eigen3/Eigen/Core>
#include <Eigen/Cholesky>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Dense>

using namespace std;

typedef Eigen::Matrix<double, 3, 1> StateVector;
typedef Eigen::Matrix<double, 3, 1> StateVectorDerivative;
typedef Eigen::Matrix<double, 3, 3> StateMatrix;

#endif

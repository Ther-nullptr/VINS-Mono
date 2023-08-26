#ifndef MATRIX_EXTRACTOR_H
#define MATRIX_EXTRACTOR_H

// #include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <fstream>

using namespace std;

// output struct
struct RawCRSMatrix
{
    std::vector<double> residuals;
    std::vector<double> gradients;
    ceres::CRSMatrix jacobian;
    ceres::CRSMatrix hessian;
};

struct RawEigenMatrix
{
    std::vector<double> residuals;
    std::vector<double> gradients;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd hessian;
};

// matrix output
void evaluateBA(ceres::Problem& problem, const ceres::Solver::Summary& summary);
Eigen::MatrixXd CRSMatrix2EigenMatrix(ceres::CRSMatrix* jacobian_crs_matrix);
ceres::CRSMatrix* EigenMatrix2CRSMatrix(Eigen::MatrixXd& jacobian_eigen_matrix);
RawEigenMatrix OutputEigenMatrix(ceres::Problem& problem);
RawCRSMatrix OutputCRSMatrix(ceres::Problem& problem);

#endif

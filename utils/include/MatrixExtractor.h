#ifndef MATRIX_EXTRACTOR_H
#define MATRIX_EXTRACTOR_H

// #include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <fstream>

using namespace std;

// matrix output
void evaluateBA(ceres::Problem& problem, const ceres::Solver::Summary& summary);
Eigen::MatrixXd CRSMatrix2EigenMatrix(ceres::CRSMatrix* jacobian_crs_matrix);

#endif

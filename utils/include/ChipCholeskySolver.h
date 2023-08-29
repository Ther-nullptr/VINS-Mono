#ifndef CHOLESKY_SOLVER_H
#define CHOLESKY_SOLVER_H

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Eigen>

struct Summary
{
    int num_iterations;
    std::string termination_type;

    double max_gradient_norm;
    double cost_change_ratio;
    double parameter_change_ratio;

    std::string brief_report();
};

template <bool chip_arithmetic, bool print_flag,
          typename in_chip_input_type, typename in_chip_output_type,
          typename residual_type, typename matrix_type>
class ChipCholeskySolver
{
public:
    ChipCholeskySolver(ceres::Problem& problem, int max_iteration, double parameter_tolerance = 1e-08, double function_tolerance = 1e-06, double gradient_tolerance = 1e-10);
    ChipCholeskySolver(const ChipCholeskySolver&) = delete;
    ChipCholeskySolver& operator=(const ChipCholeskySolver&) = delete;
    ~ChipCholeskySolver();

    Summary solve();

private:
    in_chip_output_type in_chip(in_chip_input_type a); // in chip conversion
    Eigen::Matrix<matrix_type, -1, -1> CRSMatrix2EigenMatrix(ceres::CRSMatrix *jacobian_crs_matrix); // convert sparse matrix to Eigen matrix
    void cholesky_decomposition(Eigen::Matrix<matrix_type, -1, -1> &A); // cholesky decomposition
    void cholesky_solve(Eigen::Matrix<matrix_type, -1, -1> &L, Eigen::Matrix<residual_type, -1, 1> &b); // cholesky solve

    ceres::Problem& problem_;
    int iteration_;

    double parameter_tolerance;
    double function_tolerance;
    double gradient_tolerance;
};

#endif // CHOLESKY_SOLVER_H
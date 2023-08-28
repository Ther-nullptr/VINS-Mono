#ifndef CHOLESKY_SOLVER_H
#define CHOLESKY_SOLVER_H

#include <vector>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Eigen>

template <bool chip_arithmetic = true, bool print_flag = false, 
          typename in_chip_input_type = double, typename in_chip_output_type = float
          typename residual_type = float, typename matrix_type = float>
class ChipCholeskySolver
{
public:
    ChipCholeskySolver(ceres::Problem& problem);
    ChipCholeskySolver(const ChipCholeskySolver&) = delete;
    ChipCholeskySolver& operator=(const ChipCholeskySolver&) = delete;
    ~ChipCholeskySolver();

    void solve();

private:
    in_chip_output_type in_chip(in_chip_input_type in); // in chip conversion
    Eigen::Matrix<matrix_type, -1, -1> CRSMatrix2EigenMatrix(ceres::CRSMatrix *jacobian_crs_matrix); // convert sparse matrix to Eigen matrix
    void cholesky_decomposition(Eigen::Matrix<matrix_type, -1, -1> &A); // cholesky decomposition
    void cholesky_solve(Eigen::Matrix<matrix_type, -1, -1> &L, Eigen::Matrix<residual_type, -1, -1> &b); // cholesky solve

    ceres::Problem& problem_;
    int iteration_;

    // std::vector<double *> parameter_blocks_;
    // std::vector<double> residuals_;
};

#endif // CHOLESKY_SOLVER_H
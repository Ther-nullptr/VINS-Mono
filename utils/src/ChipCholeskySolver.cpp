#include "ChipCholeskySolver.h"

std::string Summary::brief_report()
{
    std::string report;
    report += "[Brief report]\n";
    report += "Iterations: " + std::to_string(num_iterations) + "\n";
    report += "Termination type: " + termination_type + "\n";
    report += "Max gradient norm: " + std::to_string(max_gradient_norm) + "\n";
    report += "Cost change ratio: " + std::to_string(cost_change_ratio) + "\n";
    report += "Parameter change ratio: " + std::to_string(parameter_change_ratio) + "\n";
    return report;
}

template <bool chip_arithmetic, bool print_flag,
          typename in_chip_input_type, typename in_chip_output_type,
          typename residual_type, typename matrix_type>
ChipCholeskySolver<chip_arithmetic, print_flag, in_chip_input_type, in_chip_output_type, residual_type, matrix_type>::ChipCholeskySolver(ceres::Problem &problem, int max_iteration, double parameter_tolerance, double function_tolerance, double gradient_tolerance) : problem_(problem), iteration_(max_iteration), parameter_tolerance(parameter_tolerance), function_tolerance(function_tolerance), gradient_tolerance(gradient_tolerance) {}

template <bool chip_arithmetic, bool print_flag,
          typename in_chip_input_type, typename in_chip_output_type,
          typename residual_type, typename matrix_type>
ChipCholeskySolver<chip_arithmetic, print_flag, in_chip_input_type, in_chip_output_type, residual_type, matrix_type>::~ChipCholeskySolver() {}

template <bool chip_arithmetic, bool print_flag,
          typename in_chip_input_type, typename in_chip_output_type,
          typename residual_type, typename matrix_type>
in_chip_output_type ChipCholeskySolver<chip_arithmetic, print_flag, in_chip_input_type, in_chip_output_type, residual_type, matrix_type>::in_chip(in_chip_input_type a)
{
    if (a == 0)
    {
        return 0;
    }
    int exp;
    double man;
    man = std::frexp(a, &exp);
    if (exp > 128)
    {
        return (a > 0) ? 127 * std::pow(2, 121) : -127 * std::pow(2, 121);
    }
    else if (exp < -133)
    {
        return 0;
    }

    if (exp >= -127)
    {
        man = man * std::pow(2, 7);
        int tmp;
        tmp = int(man);
        return tmp * std::pow(2, exp - 7);
    }
    else
    {
        int e = exp + 134;
        man = man * std::pow(2, e);
        int tmp;
        tmp = int(man);
        return tmp * std::pow(2, -134);
    }
}

template <bool chip_arithmetic, bool print_flag,
          typename in_chip_input_type, typename in_chip_output_type,
          typename residual_type, typename matrix_type>
Eigen::Matrix<matrix_type, -1, -1> ChipCholeskySolver<chip_arithmetic, print_flag, in_chip_input_type, in_chip_output_type, residual_type, matrix_type>::CRSMatrix2EigenMatrix(ceres::CRSMatrix *jacobian_crs_matrix)
{
    Eigen::Matrix<matrix_type, -1, -1> J(jacobian_crs_matrix->num_rows, jacobian_crs_matrix->num_cols);
    J.setZero();

    std::vector<int> jacobian_crs_matrix_rows, jacobian_crs_matrix_cols;
    std::vector<matrix_type> jacobian_crs_matrix_values;
    jacobian_crs_matrix_rows = jacobian_crs_matrix->rows;
    jacobian_crs_matrix_cols = jacobian_crs_matrix->cols;
    jacobian_crs_matrix_values = jacobian_crs_matrix->values;

    int cur_index_in_cols_and_values = 0;
    // rows is a num_rows + 1 sized array
    int row_size = static_cast<int>(jacobian_crs_matrix_rows.size()) - 1;
    // outer loop traverse rows, inner loop traverse cols and values
    for (int row_index = 0; row_index < row_size; ++row_index)
    {
        while (cur_index_in_cols_and_values < jacobian_crs_matrix_rows[row_index + 1])
        {
            J(row_index, jacobian_crs_matrix_cols[cur_index_in_cols_and_values]) = jacobian_crs_matrix_values[cur_index_in_cols_and_values];
            cur_index_in_cols_and_values++;
        }
    }
    return J;
}

template <bool chip_arithmetic, bool print_flag,
          typename in_chip_input_type, typename in_chip_output_type,
          typename residual_type, typename matrix_type>
void ChipCholeskySolver<chip_arithmetic, print_flag, in_chip_input_type, in_chip_output_type, residual_type, matrix_type>::cholesky_decomposition(Eigen::Matrix<matrix_type, -1, -1> &A)
{
    int n = A.rows();
    for (int i = 0; i < n; i++)
    {
        in_chip_input_type tmp = std::sqrt(A(i, i));
        if (chip_arithmetic)
        {
            A(i, i) = in_chip(tmp);
        }
        else
        {
            A(i, i) = tmp;
        }
        for (int j = i + 1; j < n; j++)
        {
            in_chip_input_type t;
            if (A(i, i) == 0)
            {
                t = 0;
            }
            else
            {
                t = in_chip_input_type(A(j, i) / A(i, i));
            }
            if (chip_arithmetic)
            {
                A(j, i) = in_chip(t);
            }
            else
            {
                A(j, i) = t;
            }
        }
        for (int j = i + 1; j < n; j++)
        {
            for (int h = j; h < n; h++)
            {
                A(h, j) -= A(h, i) * A(j, i);
            }
        }
    }
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < i; j++)
        {
            A(j, i) = 0;
        }
    }
}

template <bool chip_arithmetic, bool print_flag,
          typename in_chip_input_type, typename in_chip_output_type,
          typename residual_type, typename matrix_type>
void ChipCholeskySolver<chip_arithmetic, print_flag, in_chip_input_type, in_chip_output_type, residual_type, matrix_type>::cholesky_solve(Eigen::Matrix<matrix_type, -1, -1> &L, Eigen::Matrix<residual_type, -1, 1> &b)
{
    int n = L.rows();
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < i; j++)
        {
            b(i) -= L(i, j) * b(j);
        }
        b(i) /= L(i, i);
    }
    for (int i = n - 1; i >= 0; i--)
    {
        for (int j = n - 1; j > i; j--)
        {
            b(i) -= L(j, i) * b(j);
        }
        b(i) /= L(i, i);
    }
}

template <bool chip_arithmetic, bool print_flag,
          typename in_chip_input_type, typename in_chip_output_type,
          typename residual_type, typename matrix_type>
Summary ChipCholeskySolver<chip_arithmetic, print_flag, in_chip_input_type, in_chip_output_type, residual_type, matrix_type>::solve()
{
    std::vector<double *> parameter_blocks;
    
    std::vector<double> residuals;
    std::vector<double> gradients;
    double prev_cost = 0, cost = 0;
    ceres::CRSMatrix jacobian_crs_matrix;
    ceres::Problem::EvaluateOptions eval_opts;

    problem_.GetParameterBlocks(&parameter_blocks);
    Eigen::Matrix<matrix_type, -1, -1> J;
    Eigen::Matrix<matrix_type, -1, -1> H;

    // get the basic info of problem
    if (print_flag)
    {
        std::cout << "[Number of parameter blocks]: " << problem_.NumParameterBlocks() << std::endl;
        std::cout << "[Number of parameters]: " << problem_.NumParameters() << std::endl;
        std::cout << "[Number of residual blocks]: " << problem_.NumResidualBlocks() << std::endl;
        std::cout << "[Number of residuals]: " << problem_.NumResiduals() << std::endl;
    }

    for (int i = 0; i < iteration_; i++)
    {
        // deep copy parameter_blocks to prev_parameter_blocks

        prev_cost = cost;

        problem_.Evaluate(eval_opts, &cost, &residuals, &gradients, &jacobian_crs_matrix);

        // get J and H
        J = CRSMatrix2EigenMatrix(&jacobian_crs_matrix);
        H = J.transpose() * J;
        
        // cholesky decomposition
        cholesky_decomposition(H);

        // get b
        Eigen::Matrix<residual_type, -1, 1> b = Eigen::Map<Eigen::Matrix<residual_type, -1, 1>>(gradients.data(), gradients.size());

        // solve
        cholesky_solve(H, b);

        std::vector<double> prev_parameter_blocks;
        for (auto it = parameter_blocks.begin(); it != parameter_blocks.end(); it++)
        {
            prev_parameter_blocks.push_back(*(*it));
        }

        // update x
        for (auto it = parameter_blocks.begin(); it != parameter_blocks.end(); it++)
        {
            *(*it) -= b(it - parameter_blocks.begin());
        }

        // if (print_flag)
        // {
        //     std::cout << "[iteration: " << i << "]" << std::endl;
        //     double max_gradient_norm = std::abs(*std::max_element(gradients.begin(), gradients.end(), [](double a, double b)
        //                                                 { return std::abs(a) < std::abs(b); }));
        //     std::cout << "max gradient norm: ";
        //     std::cout << max_gradient_norm << std::endl;
        //     std::cout << "|cost_change|/cost: " << std::abs(cost - prev_cost) / prev_cost << std::endl;

        //     // get the change of parameter_blocks
        //     std::vector<double> parameter_blocks_change;
        //     for (auto it = parameter_blocks.begin(); it != parameter_blocks.end(); it++)
        //     {
        //         parameter_blocks_change.push_back(*(*it) - prev_parameter_blocks[it - parameter_blocks.begin()]);
        //     }
        //     // compute the norm of parameter_blocks_change
        //     double parameter_blocks_change_norm = std::sqrt(std::inner_product(parameter_blocks_change.begin(), parameter_blocks_change.end(), parameter_blocks_change.begin(), 0.0));
        //     // std::cout << "parameter_blocks_change_norm: " << parameter_blocks_change_norm << std::endl;
        //     double parameter_blocks_norm = std::sqrt(std::inner_product(prev_parameter_blocks.begin(), prev_parameter_blocks.end(), prev_parameter_blocks.begin(), 0.0));
        //     // std::cout << "parameter_blocks_norm: " << parameter_blocks_norm << std::endl;
        //     std::cout << "norm ratio: " << parameter_blocks_change_norm / parameter_blocks_norm << std::endl;
        //     std::cout << "x: ";
        //     for (auto it = parameter_blocks.begin(); it != parameter_blocks.end(); it++)
        //     {
        //         std::cout << *(*it) << " ";
        //     }
        //     std::cout << std::endl;
        // }

        double max_gradient_norm = std::abs(*std::max_element(gradients.begin(), gradients.end(), [](double a, double b)
                                                    { return std::abs(a) < std::abs(b); }));
        double cost_change_ratio = std::abs(cost - prev_cost) / prev_cost;
        
        std::vector<double> parameter_blocks_change;
        for (auto it = parameter_blocks.begin(); it != parameter_blocks.end(); it++)
        {
            parameter_blocks_change.push_back(*(*it) - prev_parameter_blocks[it - parameter_blocks.begin()]);
        }
        double parameter_blocks_change_norm = std::sqrt(std::inner_product(parameter_blocks_change.begin(), parameter_blocks_change.end(), parameter_blocks_change.begin(), 0.0));
        double prev_parameter_blocks_norm = std::sqrt(std::inner_product(prev_parameter_blocks.begin(), prev_parameter_blocks.end(), prev_parameter_blocks.begin(), 0.0));
        double parameter_change_ratio = parameter_blocks_change_norm / prev_parameter_blocks_norm;

        if (max_gradient_norm < gradient_tolerance)
        {
            Summary summary{.num_iterations = i,
                            .termination_type = "Gradient tolerance reached",
                            .max_gradient_norm = max_gradient_norm,
                            .cost_change_ratio = cost_change_ratio,
                            .parameter_change_ratio = parameter_change_ratio};
            return summary;
        }
        else if (cost_change_ratio < function_tolerance)
        {
            Summary summary{.num_iterations = i,
                            .termination_type = "Function tolerance reached",
                            .max_gradient_norm = max_gradient_norm,
                            .cost_change_ratio = cost_change_ratio,
                            .parameter_change_ratio = parameter_change_ratio};
            return summary;
        }
        else if (parameter_change_ratio < parameter_tolerance)
        {
            Summary summary{.num_iterations = i,
                            .termination_type = "Parameter tolerance reached",
                            .max_gradient_norm = max_gradient_norm,
                            .cost_change_ratio = cost_change_ratio,
                            .parameter_change_ratio = parameter_change_ratio};
            return summary;
        }

        if (i == iteration_ - 1)
        {
            Summary summary{.num_iterations = i,
                            .termination_type = "Maximum iterations reached",
                            .max_gradient_norm = max_gradient_norm,
                            .cost_change_ratio = cost_change_ratio,
                            .parameter_change_ratio = parameter_change_ratio};
            
            return summary;
        }
    }
}

template class ChipCholeskySolver<true, true, double, double, double, double>;
template class ChipCholeskySolver<false, true, double, double, double, double>;

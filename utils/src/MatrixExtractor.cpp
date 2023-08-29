#include "../include/MatrixExtractor.h"

const char* OUTPUT_PATH = "/output";

void evaluateBA(ceres::Problem& problem, const ceres::Solver::Summary& summary){
    cout << summary.FullReport() << endl;

    ceres::Problem::EvaluateOptions EvalOpts;
    ceres::CRSMatrix jacobian_crs_matrix;
    std::vector<double> residuals;
    std::vector<double> gradients;
    problem.Evaluate(EvalOpts, nullptr, &residuals, &gradients, &jacobian_crs_matrix);

    //TicToc t_convert_J;
    Eigen::MatrixXd J = CRSMatrix2EigenMatrix(&jacobian_crs_matrix);
    //cout << "convert sparse matrix cost " << t_convert_J.toc() << endl;

    //TicToc t_construct_H;
    Eigen::MatrixXd H = J.transpose()*J;
    std::cout << H << std::endl;
    //cout << "construct H cost " << t_construct_H.toc() << endl;

    static ofstream J_file("/home/jovyan/output/jacobian.txt");
    J_file << fixed;
    J_file.precision(9);

    int J_rows = J.rows(), J_cols = J.cols();
    for(int i = 0; i < J_rows; ++i){
        for(int j = 0;j < J_cols; ++j){
            J_file << J(i, j);
            if(j == J_cols - 1){
                J_file << endl;
            }
            else{
                J_file << " ";
            }
        }
    }

    J_file << "===================\n";

    static ofstream H_file("/home/jovyan/output/hessian.txt");
    H_file << fixed;
    H_file.precision(9);

    int H_rows = H.rows(), H_cols = H.cols();
    for(int i = 0; i < H_rows; ++i){
        for(int j = 0; j < H_cols; ++j){
            H_file << H(i, j);
            if(j == H_cols - 1){
                H_file << endl;
            }
            else{
                H_file << " ";
            }
        }
    }

    H_file << "===================\n";

    static ofstream residual_file("/home/jovyan/output/residual.txt");
    residual_file << fixed;
    residual_file.precision(9);

    for(int i = 0; i < residuals.size(); i++)
    {
        residual_file << residuals[i] << endl;
    }
    residual_file << "===================\n";

    static ofstream gradient_file("/home/jovyan/output/gradient.txt");
    gradient_file << fixed;
    gradient_file.precision(9);

    for(int i = 0; i < gradients.size(); i++)
    {
        gradient_file << gradients[i] << endl;
    }
    gradient_file << "===================\n";
}

RawEigenMatrix OutputEigenMatrix(ceres::Problem& problem)
{
    ceres::Problem::EvaluateOptions EvalOpts;
    ceres::CRSMatrix jacobian_crs_matrix;
    std::vector<double> residuals;
    std::vector<double> gradients;
    problem.Evaluate(EvalOpts, nullptr, &residuals, &gradients, &jacobian_crs_matrix);

    Eigen::MatrixXd J = CRSMatrix2EigenMatrix(&jacobian_crs_matrix);
    Eigen::MatrixXd H = J.transpose() * J;

    RawEigenMatrix result{residuals, gradients, J, H};

    return result;
}

// RawCRSMatrix OutputCRSMatrix(ceres::Problem& problem)
// {
//     ceres::Problem::EvaluateOptions EvalOpts;
//     ceres::CRSMatrix jacobian_crs_matrix;
//     std::vector<double> residuals;
//     std::vector<double> gradients;
//     problem.Evaluate(EvalOpts, nullptr, &residuals, &gradients, &jacobian_crs_matrix);

//     Eigen::MatrixXd J = CRSMatrix2EigenMatrix(&jacobian_crs_matrix);
//     Eigen::MatrixXd H = J.transpose() * J;

//     ceres::CRSMatrix hessian_crs_matrix = EigenMatrix2CRSMatrix(H);

//     RawCRSMatrix result{residuals, gradients, jacobian_crs_matrix, hessian_crs_matrix};

//     return result;
// }

Eigen::MatrixXd CRSMatrix2EigenMatrix(ceres::CRSMatrix* jacobian_crs_matrix){
    Eigen::MatrixXd J(jacobian_crs_matrix->num_rows, jacobian_crs_matrix->num_cols);
    J.setZero();

    std::vector<int> jacobian_crs_matrix_rows, jacobian_crs_matrix_cols;
    std::vector<double> jacobian_crs_matrix_values;
    jacobian_crs_matrix_rows = jacobian_crs_matrix->rows;
    jacobian_crs_matrix_cols = jacobian_crs_matrix->cols;
    jacobian_crs_matrix_values = jacobian_crs_matrix->values;

    int cur_index_in_cols_and_values = 0;
    // rows is a num_rows + 1 sized array
    int row_size = static_cast<int>(jacobian_crs_matrix_rows.size()) - 1;
    // outer loop traverse rows, inner loop traverse cols and values
    for(int row_index = 0; row_index < row_size; ++row_index){
        while(cur_index_in_cols_and_values < jacobian_crs_matrix_rows[row_index+1]){
            J(row_index, jacobian_crs_matrix_cols[cur_index_in_cols_and_values]) = jacobian_crs_matrix_values[cur_index_in_cols_and_values];
            cur_index_in_cols_and_values++;
        }
    }
    return J;
}

ceres::CRSMatrix* EigenMatrix2CRSMatrix(Eigen::MatrixXd& jacobian_eigen_matrix) {
    int num_rows = jacobian_eigen_matrix.rows();
    int num_cols = jacobian_eigen_matrix.cols();

    // Create vectors to hold CRS matrix data
    std::vector<int> rows(num_rows + 1, 0);
    std::vector<int> cols;
    std::vector<double> values;

    for (int row_index = 0; row_index < num_rows; ++row_index) {
        for (int col_index = 0; col_index < num_cols; ++col_index) {
            double value = jacobian_eigen_matrix(row_index, col_index);
            if (value != 0.0) {
                cols.push_back(col_index);
                values.push_back(value);
            }
        }
        rows[row_index + 1] = static_cast<int>(cols.size());
    }

    // Create a CRSMatrix and assign values
    ceres::CRSMatrix* jacobian_crs_matrix = new ceres::CRSMatrix;
    jacobian_crs_matrix->num_rows = num_rows;
    jacobian_crs_matrix->num_cols = num_cols;
    jacobian_crs_matrix->rows = rows;
    jacobian_crs_matrix->cols = cols;
    jacobian_crs_matrix->values = values;

    return jacobian_crs_matrix;
}

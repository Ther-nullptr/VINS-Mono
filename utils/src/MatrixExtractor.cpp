#include "../include/MatrixExtractor.h"

const char* OUTPUT_PATH = "/home/jovyan/output/MH_01";

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
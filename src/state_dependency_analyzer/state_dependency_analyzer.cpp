//StateDependencyAnalyzer.cpp
#include "state_dependency_analyzer.h"

std::vector<OBSERVABILITY> obs_data_buff_;
std::string sub_motion_state;
std::string motion_state;
std::string folder_suffix;

void UpdateGY(const std::string& fusion_strategy) {
    if (fusion_strategy == "gnss_odom_lidar") {
        GetGYPosiPoseVel();
    } else if (fusion_strategy == "gnss_lidar") {
        GetGYPosiPose();
    } else if (fusion_strategy == "gnss_odom") {
        GetGYPosiVel();
    } else if (fusion_strategy == "gnss") {
        GetGYPosi();
    } else if (fusion_strategy == "lidar") {
        GetGYPose();
    } else {
        std::cout << "Unknown fusion strategy: " << fusion_strategy << std::endl;
    }
}

void PerformSVDAndObservability(int Ysize,
                                const std::string &obs_result_path)
{
    Eigen::MatrixXd Qsom;
    Eigen::VectorXd Ysom;

    BuildQsomYsom(Qsom, Ysom, Ysize);

    Eigen::MatrixXd QsomProcessed;
    Eigen::VectorXd YsomProcessed;

    std::vector<int> relevantObsIndices;

    ProcessingData part = ProcessingData::All;
    int rowsPerInstance = Qsom.rows() / obs_data_buff_.size();

    std::cout << "Processing part: " << static_cast<int>(part) + 1 << std::endl;
    switch(part) {
        case ProcessingData::FirstHalf: {
            int halfSize = Qsom.rows() / 2;
            QsomProcessed = Qsom.topRows(halfSize);
            YsomProcessed = Ysom.head(halfSize);
            for (int i = 0; i < halfSize / rowsPerInstance; ++i) {
                relevantObsIndices.push_back(i);
            }
            break;
        }
        case ProcessingData::SecondHalf: {
            int halfSize = Qsom.rows() / 2;
            QsomProcessed = Qsom.bottomRows(halfSize);
            YsomProcessed = Ysom.tail(halfSize);
            for (int i = halfSize / rowsPerInstance; i < obs_data_buff_.size(); ++i) {
                relevantObsIndices.push_back(i);
            }
            break;
        }
        case ProcessingData::All:
        default:
            QsomProcessed = Qsom;
            YsomProcessed = Ysom;
            for (int i = 0; i < obs_data_buff_.size(); ++i) {
                relevantObsIndices.push_back(i);
            }
            break;
    }

    Eigen::MatrixXd U, V;
    Eigen::VectorXd S;

    PerformSVD(QsomProcessed, U, V, S, relevantObsIndices);

    Eigen::MatrixXd X;
    ComputeObservationMatrixX(U, V, S, YsomProcessed, X);

    Eigen::VectorXi maxIndices;
    FindMaxIndicesInX(X, maxIndices);

    Eigen::VectorXd observability_score;
    MapSingularValuesToStates(S, maxIndices, observability_score);
    
    Eigen::VectorXd normalized_observability_score;
    NormalizeObservabilityScore(S, observability_score, normalized_observability_score, maxIndices, sub_motion_state, obs_result_path);

}

void BuildQsomYsom(Eigen::MatrixXd &Qsom, Eigen::VectorXd &Ysom, int Ysize) {
    int obs_size = obs_data_buff_.size();
    std::cout << "obs_size: " << obs_size << std::endl;
    int totalRowsQ = 0;
    Eigen::Matrix<double, 15, 15> F_accumulate;

    for (const auto& observability : obs_data_buff_) {
        totalRowsQ += Ysize * observability.G.size();
    }

    Qsom = Eigen::MatrixXd(totalRowsQ, 15);
    Ysom = Eigen::VectorXd(totalRowsQ);
    int currentRow = 0;
    for (int i = 0; i < obs_size; ++i) {
        const auto& currentObservability = obs_data_buff_[i];
        F_accumulate = Eigen::Matrix<double, 15, 15>::Identity();
        Qsom.block(currentRow, 0, Ysize, 15) = currentObservability.G[0] * F_accumulate; 
        Ysom.segment(currentRow, Ysize) = currentObservability.Y[0];
        currentRow += Ysize;

        for (size_t j = 1; j < currentObservability.G.size(); ++j) {
            F_accumulate *= Ft - Eigen::Matrix<double, 15, 15>::Identity();
            Qsom.block(currentRow, 0, Ysize, 15) = currentObservability.G[j] * F_accumulate;
            Ysom.segment(currentRow, Ysize) = currentObservability.Y[j];
            currentRow += Ysize;
        }
    }
    std::cout << "Qsom size: " << Qsom.rows() << " x " << Qsom.cols() << std::endl;
    std::cout << "Ysom size: " << Ysom.rows() << std::endl;
}

void PerformSVD(const Eigen::MatrixXd &Qsom, 
                Eigen::MatrixXd &U, 
                Eigen::MatrixXd &V, 
                Eigen::VectorXd &S,
                const std::vector<int>& relevantObsIndices)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Qsom, Eigen::ComputeThinU | Eigen::ComputeThinV);
    U = svd.matrixU();
    V = svd.matrixV();
    S = svd.singularValues();

    for (int index : relevantObsIndices) {
    std::cout << "Processing observability instance [" << obs_data_buff_[index].index  << "]" << std::endl;

    }
}

void ComputeObservationMatrixX(const Eigen::MatrixXd &U, 
                               const Eigen::MatrixXd &V, 
                               const Eigen::VectorXd &S, 
                               const Eigen::VectorXd &Ysom, 
                               Eigen::MatrixXd &X)
{
    Eigen::MatrixXd S_inv = S.asDiagonal().inverse();
    Eigen::VectorXd U_trans_Ysom = U.transpose() * Ysom;
    Eigen::MatrixXd U_trans_Ysom_diag = U_trans_Ysom.asDiagonal();
    X = V * S_inv * U_trans_Ysom_diag;
}

void FindMaxIndicesInX(const Eigen::MatrixXd &X, 
                       Eigen::VectorXi &maxIndices)
{
    maxIndices = Eigen::VectorXi::Zero(X.cols());

    for (int i = 0; i < X.cols(); ++i)
    {
        Eigen::MatrixXd::Index maxIndex;
        X.col(i).cwiseAbs().maxCoeff(&maxIndex);
        maxIndices(i) = maxIndex;
    }
}

void MapSingularValuesToStates(const Eigen::VectorXd &S, 
                               const Eigen::VectorXi &maxIndices, 
                               Eigen::VectorXd &observability_score)
{
    observability_score = Eigen::VectorXd::Zero(S.size());

    for (int i = 0; i < maxIndices.size(); ++i)
    {
        observability_score(maxIndices(i)) = S(i);
    }
}

void NormalizeObservabilityScore(const Eigen::VectorXd &S, 
                                 const Eigen::VectorXd &observability_score,
                                 Eigen::VectorXd &normalized_observability_score, 
                                 const Eigen::VectorXi &maxIndices, 
                                 const std::string &sub_motion_state, 
                                 const std::string &obs_result_path)
{
    normalized_observability_score = observability_score / S(0);

    std::ofstream writeObsToFile(obs_result_path, std::ios_base::app);

    if (writeObsToFile.fail())
    {
        std::cerr << "Error: Could not open the file for writing." << std::endl;
        return;
    }

    writeObsToFile << std::left << "sub_motion_state: " << std::setw(10) << sub_motion_state << std::endl;

    for (int i = 0; i < maxIndices.size(); ++i)
    {
        writeObsToFile << "State " << std::setw(2) << maxIndices(i)
                       << ", SingularValue: " << std::setw(10) << std::setprecision(5) << S(i)
                       << ", Observability: " << std::setw(12) << std::setprecision(11)
                       << normalized_observability_score(maxIndices(i)) << std::endl;
    }

    writeObsToFile.close();

    if (writeObsToFile.fail())
    {
        std::cerr << "Error: Could not write to the file successfully." << std::endl;
        return;
    }
}

int ComputeYSize(const std::string &fusion_strategy)
{
    if (fusion_strategy == "gnss") // gnss_xyz
    {
        return 3;
    }
    else if (fusion_strategy == "gnss_odom" || fusion_strategy == "lidar") // gnss_xyz, gnss_v
    {
        return 6;
    }
    else if (fusion_strategy == "gnss_lidar") // gnss_xyz, lidar_xyz, lidar_yaw
    {
        return 9;
    }
    else if (fusion_strategy == "gnss_odom_lidar") // gnss_xyz, gnss_v, lidar_xyz, lidar_yaw
    {
        return 12;
    }
    else
    {
        return 3; // gnss_xyz
    }
}


bool GetGYPosiPoseVel() {
    if (!obs_data_buff_.empty() && obs_data_buff_.back().G.size() != 15) {
        obs_data_buff_.back().G.push_back(GPosiPoseVel);
        obs_data_buff_.back().Y.push_back(YPosiPoseVel);
        return true;
    }

    OBSERVABILITY observability;
    std::stringstream time_label;
    observability.time = current_ref_data_.time;
    time_label << std::fixed << std::setprecision(1) << observability.time;
    observability.index = "Time: " + time_label.str();
    observability.G.push_back(GPosiPoseVel);
    observability.Y.push_back(YPosiPoseVel);
    obs_data_buff_.push_back(observability);

    return true;
}

bool GetGYPosiPose() {
    if (!obs_data_buff_.empty() && obs_data_buff_.back().G.size() != 15) {
        obs_data_buff_.back().G.push_back(GPosiPose);
        obs_data_buff_.back().Y.push_back(YPosiPose);
        return true;
    }

    OBSERVABILITY observability;
    std::stringstream time_label;
    observability.time = current_ref_data_.time;
    time_label << std::fixed << std::setprecision(1) << observability.time;
    observability.index = "Time: " + time_label.str();
    observability.G.push_back(GPosiPose);
    observability.Y.push_back(YPosiPose);
    obs_data_buff_.push_back(observability);

    return true;
}

bool GetGYPosiVel() {
    if (!obs_data_buff_.empty() && obs_data_buff_.back().G.size() != 15) {
        obs_data_buff_.back().G.push_back(GPosiVel);
        obs_data_buff_.back().Y.push_back(YPosiVel);
        return true;
    }

    OBSERVABILITY observability;
    std::stringstream time_label;
    observability.time = current_ref_data_.time;
    time_label << std::fixed << std::setprecision(1) << observability.time;
    observability.index = "Time: " + time_label.str();
    observability.G.push_back(GPosiVel);
    observability.Y.push_back(YPosiVel);
    obs_data_buff_.push_back(observability);

    return true;
}


bool GetGYPosi() {
    if (!obs_data_buff_.empty() && obs_data_buff_.back().G.size() != 15) {
        obs_data_buff_.back().G.push_back(GPosi);
        obs_data_buff_.back().Y.push_back(YPosi);
        return true;
    }

    OBSERVABILITY observability;
    std::stringstream time_label;
    observability.time = current_ref_data_.time;
    time_label << std::fixed << std::setprecision(1) << observability.time;
    observability.index = "Time: " + time_label.str();
    observability.G.push_back(GPosi);
    observability.Y.push_back(YPosi);
    obs_data_buff_.push_back(observability);

    return true;
}

bool GetGYPose() {
    if (!obs_data_buff_.empty() && obs_data_buff_.back().G.size() != 15) {
        obs_data_buff_.back().G.push_back(GPose);
        obs_data_buff_.back().Y.push_back(YPose);
        return true;
    }

    OBSERVABILITY observability;
    std::stringstream time_label;
    observability.time = current_ref_data_.time;
    time_label << std::fixed << std::setprecision(1) << observability.time;
    observability.index = "Time: " + time_label.str();
    observability.G.push_back(GPose);
    observability.Y.push_back(YPose);
    obs_data_buff_.push_back(observability);

    return true;
}
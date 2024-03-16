//StateDependencyAnalyzer.h
#ifndef STATE_DEPENDENCY_ANALYZER_H
#define STATE_DEPENDENCY_ANALYZER_H

#include "../app/utilityFunctions.h"
// observability analysis
struct OBSERVABILITY
{
    double time;
    std::string index;
    std::vector<Eigen::MatrixXd> G;
    std::vector<Eigen::VectorXd> Y;
};

enum class ProcessingData {
    FirstHalf,
    SecondHalf,
    All
};

extern std::vector<OBSERVABILITY> obs_data_buff_;


// motion state
extern std::string sub_motion_state;
extern std::string motion_state;
extern std::string folder_suffix;

int ComputeYSize(const std::string &fusion_strategy);
void UpdateGY(const std::string& fusion_strategy);
bool GetGYPosiPoseVel();
bool GetGYPosiPose();
bool GetGYPosiVel();
bool GetGYPosi();
bool GetGYPose();

void PerformSVDAndObservability(int Ysize,
                                const std::string &obs_result_path);
void BuildQsomYsom(Eigen::MatrixXd &Qsom,
                          Eigen::VectorXd &Ysom,
                          int Ysize);
void PerformSVD(const Eigen::MatrixXd &Qsom,
                Eigen::MatrixXd &U,
                Eigen::MatrixXd &V,
                Eigen::VectorXd &S,
                const std::vector<int>& relevantObsIndices);
void ComputeObservationMatrixX(const Eigen::MatrixXd &U,
                               const Eigen::MatrixXd &V,
                               const Eigen::VectorXd &S,
                               const Eigen::VectorXd &Y,
                               Eigen::MatrixXd &X);
void FindMaxIndicesInX(const Eigen::MatrixXd &X,
                       Eigen::VectorXi &maxIndices);
void MapSingularValuesToStates(const Eigen::VectorXd &S,
                               const Eigen::VectorXi &maxIndices,
                               Eigen::VectorXd &observability_score);
void NormalizeObservabilityScore(const Eigen::VectorXd &S, 
                                 const Eigen::VectorXd &observability_score,
                                 Eigen::VectorXd &normalized_observability_score, 
                                 const Eigen::VectorXi &maxIndices, 
                                 const std::string &sub_motion_state, 
                                 const std::string &obs_result_path);


#endif // STATE_DEPENDENCY_ANALYZER_H
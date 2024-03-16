//error_state_kalman_filter.h
#ifndef ERROR_STATE_KALMAN_FILTER_H
#define ERROR_STATE_KALMAN_FILTER_H

#include "../app/utilityFunctions.h"

class UtilityFunctions; // forward declaration
extern UtilityFunctions utilityFunctions;

// lidar_lastest
struct LidarTemp
{
    double time;
    Eigen::Vector3d position;
};

// process noise
extern double gyro_noise;
extern double acc_noise;
extern double gyro_bias_noise;
extern double acc_bias_noise;

// measurement noise
extern double gnss_posi_nosie;
extern double lidar_posi_noise;
extern double lidar_ori_noise;
extern double odom_vel_noise;

// Update
extern Eigen::Vector3d earthRotation;

// error state kalman filter matrix
extern Eigen::Matrix<double, 15, 15> Ft;
extern Eigen::Matrix<double, 15, 12> Bt;

// measurement equation G
extern Eigen::MatrixXd GPosi;
extern Eigen::MatrixXd GPose;
extern Eigen::MatrixXd GPosiVel;
extern Eigen::MatrixXd GPosiPose;
extern Eigen::MatrixXd GPosiPoseVel;

// measurement 
extern Eigen::VectorXd YPosi;
extern Eigen::VectorXd YPose;
extern Eigen::VectorXd YPosiVel;
extern Eigen::VectorXd YPosiPose;
extern Eigen::VectorXd YPosiPoseVel;

extern SpoofData current_spoof_pose;
// Update
void ExecuteFilterAndSaveData(int &counter_for_while);
bool ErrorStateKalmanFilter();
bool Update();
bool UpdateOdomEstimation();
Eigen::Vector3d CalcTotalAngVel(const Eigen::Vector3d &currentVelocity, 
                                              const Eigen::Vector3d &currentGNSSData);
void UpdateOrientation(Eigen::Quaterniond &currentOrientation, 
                       const Eigen::Vector3d &w_in_n);
Eigen::Quaterniond CalculateRotation(const Eigen::Vector3d &angular_velocity, double dt);
bool UpdateVelAndPosi(Eigen::Vector3d &currentPosition,
                               Eigen::Vector3d &currentVelocity,
                               Eigen::Quaterniond &currentOrientation);

bool UpdateErrorEstimation();
void UpdateProcessEquation(const Eigen::Vector3d &ff,
                const Eigen::Vector3d &w_ie_n);
void UpdateCovariance(Eigen::Matrix<double, 15, 12> &Bt, 
                      Eigen::Matrix<double, 15, 1> &X_, 
                      Eigen::Matrix<double, 15, 15> &P_, 
                      const double T);
// Correct
void HandleGNSSData(Eigen::Vector3d &gnss_xyz);
bool Correct(Eigen::Vector3d &gnss_xyz,
             const std::string &sub_motion_state);
bool HandleGnssStrategy(Eigen::Vector3d &gnss_xyz,
                         const std::string &sub_motion_state,
                         const std::string &fusion_strategy);
bool HandleGnssOdomStrategy(Eigen::Vector3d &gnss_xyz,
                             const std::string &sub_motion_state,
                             const std::string &fusion_strategy);
bool HandleLidarStrategy(Eigen::Vector3d &gnss_xyz,
                          const std::string &sub_motion_state,
                          const std::string &fusion_strategy);
bool HandleGnssLidarStrategy(Eigen::Vector3d &gnss_xyz,
                              const std::string &sub_motion_state,
                              const std::string &fusion_strategy);
bool HandleGnssOdomLidarStrategy(Eigen::Vector3d &gnss_xyz,
                                  const std::string &sub_motion_state,
                                  const std::string &fusion_strategy);
void UpdateKalmanGain(Eigen::VectorXd &Y,
                      Eigen::MatrixXd &Gt,
                      Eigen::MatrixXd &Ct,
                      Eigen::MatrixXd &R,
                      const std::string &sub_motion_state);
void EliminateError();

void ResetState();

bool ChiSquaredTest(const Eigen::VectorXd &Y,
                    const Eigen::MatrixXd &Gt,
                    const Eigen::MatrixXd &P,
                    const Eigen::MatrixXd &R,
                    int degrees_of_freedom,
                    double chi_squared_critical);



#endif // ERROR_STATE_KALMAN_FILTER_H
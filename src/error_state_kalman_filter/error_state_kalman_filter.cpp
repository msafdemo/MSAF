//error_state_kalman_filter.cpp
#include "error_state_kalman_filter.h"

// process noise
double gyro_noise = 1e-6;
double acc_noise = 1e-6;
double gyro_bias_noise = 1e-6;
double acc_bias_noise = 1e-6;
// measurement noise
double lidar_posi_noise = 1e-6;
double lidar_ori_noise=1e-6;
double odom_vel_noise = 1e-6;
double gnss_posi_nosie = 1e-6;

SpoofData current_spoof_pose;
LidarTemp lidartemp;

// predict
Eigen::Vector3d w_ie_n;

// error state kalman filter matrix
Eigen::Matrix<double, 15, 15> Ft = Eigen::Matrix<double, 15, 15>::Zero();
Eigen::Matrix<double, 15, 12> Bt = Eigen::Matrix<double, 15, 12>::Zero();

// measurement equation G
Eigen::MatrixXd GPosi = Eigen::Matrix<double, 3, 15>::Zero();
Eigen::MatrixXd GPose = Eigen::Matrix<double, 6, 15>::Zero();
Eigen::MatrixXd GPosiVel = Eigen::Matrix<double, 6, 15>::Zero();
Eigen::MatrixXd GPosiPose = Eigen::Matrix<double, 9, 15>::Zero();
Eigen::MatrixXd GPosiPoseVel = Eigen::Matrix<double, 12, 15>::Zero();

// measurement Y
Eigen::VectorXd YPosi = Eigen::Matrix<double, 3, 1>::Zero();
Eigen::VectorXd YPose = Eigen::Matrix<double, 6, 1>::Zero();
Eigen::VectorXd YPosiVel = Eigen::Matrix<double, 6, 1>::Zero();
Eigen::VectorXd YPosiPose = Eigen::Matrix<double, 9, 1>::Zero();
Eigen::VectorXd YPosiPoseVel = Eigen::Matrix<double, 12, 1>::Zero();

// Class Instances
UtilityFunctions utilityFunctions;

void ExecuteFilterAndSaveData(int &counter_for_while)
{
    while (ValidData(true))
    {
        counter_for_while++;        
        ErrorStateKalmanFilter();
        utilityFunctions.SaveData();
        UpdateGY(fusion_strategy);
        // GetGYPosiPoseVel();   
    }
}

bool ErrorStateKalmanFilter()
{
    Update();
    predictCounter++;
    Eigen::Vector3d gnss_xyz;
    HandleGNSSData(gnss_xyz);
    Correct(gnss_xyz, sub_motion_state);
    correctCounter++;
    return true;
}

bool Update()
{
    if (!UpdateOdomEstimation())
    {
        return false;
    }

    if (!UpdateErrorEstimation())
    {
        return false;
    }

    return true;
}

bool UpdateOdomEstimation()
{
    current_pose.time = current_ref_data_.time;
    Eigen::Vector3d currentPosition = current_pose.state.p;
    Eigen::Vector3d currentVelocity = current_pose.state.v;
    Eigen::Quaterniond currentOrientation = current_pose.state.q;

    Eigen::Vector3d w_in_n = CalcTotalAngVel(currentVelocity, current_gnss_data_.lla);

    UpdateOrientation(currentOrientation, w_in_n);

    if (!UpdateVelAndPosi(currentPosition, currentVelocity, currentOrientation))
    {
        return false;
    }

    current_pose.state.p = currentPosition;
    current_pose.state.v = currentVelocity;
    current_pose.state.q = currentOrientation;

    return true;
}

Eigen::Vector3d CalcTotalAngVel(const Eigen::Vector3d &currentVelocity, const Eigen::Vector3d &currentGNSSData)
{
    const double w = 7.27220521664304e-05;
    const double rm = 6353346.18315;
    const double rn = 6384140.52699;
    double cosLat = std::cos(currentGNSSData[0] * DEG_TO_RAD);
    double sinLat = std::sin(currentGNSSData[0] * DEG_TO_RAD);

    Eigen::Vector3d w_ie_n(0, w * cosLat, w * sinLat);

    double velOverRmH = currentVelocity[1] / (rm + currentGNSSData[2]);
    double velOverRnH = currentVelocity[0] / (rn + currentGNSSData[2]);

    Eigen::Vector3d w_en_n(-velOverRmH, 
                            velOverRnH, 
                            velOverRnH * std::tan(currentGNSSData[0] * DEG_TO_RAD));

    return w_ie_n + w_en_n;
}

void UpdateOrientation(Eigen::Quaterniond &currentOrientation, const Eigen::Vector3d &w_in_n)
{
    for (int i = 1; i < current_imu_data_.size(); ++i)
    {
        double dt = current_imu_data_[i].time - current_imu_data_[i - 1].time;

        Eigen::Quaterniond qn = CalculateRotation(w_in_n, dt);

        Eigen::Vector3d wb = 0.5 * (current_imu_data_[i - 1].gyro + current_imu_data_[i].gyro) + current_pose.state.bg;
        Eigen::Quaterniond qb = CalculateRotation(wb, dt);

        currentOrientation = qn.inverse() * currentOrientation * qb; 
    }
}

Eigen::Quaterniond CalculateRotation(const Eigen::Vector3d &angular_velocity, double dt)
{
    Eigen::Vector3d theta = angular_velocity * dt;
    double angle = theta.norm();

    if (angle > std::numeric_limits<double>::epsilon())
    {
        Eigen::Vector3d axis = theta.normalized(); 
        return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    }
    else
    {
        return Eigen::Quaterniond::Identity();
    }
}

bool UpdateVelAndPosi(Eigen::Vector3d &currentPosition, 
                               Eigen::Vector3d &currentVelocity, 
                               Eigen::Quaterniond &currentOrientation)
{
    Eigen::Vector3d gravity(0, 0, -9.79484197226504);

    for (int i = 1; i < current_imu_data_.size(); ++i)
    {
        double deltaT = current_imu_data_[i].time - current_imu_data_[i - 1].time;

        Eigen::Vector3d acc_b_1 = current_imu_data_[i - 1].acc + current_pose.state.ba;
        Eigen::Vector3d acc_b_2 = current_imu_data_[i].acc + current_pose.state.ba;

        Eigen::Vector3d acc_n_1 = currentOrientation * acc_b_1;
        Eigen::Vector3d acc_n_2 = currentOrientation * acc_b_2;
        Eigen::Vector3d avgAcc_n = 0.5 * (acc_n_1 + acc_n_2) + gravity;

        Eigen::Vector3d updatedVelocity = currentVelocity + deltaT * avgAcc_n;
        Eigen::Vector3d updatedPosition = currentPosition + 0.5 * deltaT * (currentVelocity + updatedVelocity);

        currentPosition = updatedPosition;
        currentVelocity = updatedVelocity;
    }
    return true;
}

bool UpdateErrorEstimation()
{
    Eigen::Vector3d acc_n = current_imu_data_.back().acc;
    acc_n = current_pose.state.q * acc_n;

    UpdateProcessEquation(acc_n, w_ie_n);

    double T = current_imu_data_.back().time - current_imu_data_.front().time;
    UpdateCovariance(Bt, current_error_state.X_, current_error_state.P_, T);

    return true;
}

void UpdateProcessEquation(const Eigen::Vector3d &acc_n, 
                const Eigen::Vector3d &w_ie_n) {
    Eigen::Matrix3d C_nb = current_pose.state.q.toRotationMatrix();
    Bt = Eigen::Matrix<double, 15, 12>::Zero();
    // F Matrix for delta posi
    Ft.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    // F Matrix for delta vel
    Ft.block<3, 3>(3, 6) = Sophus::SO3d::hat(acc_n).matrix();;
    Ft.block<3, 3>(3, 12) = C_nb;
    // F Matrix for delta ori
    Ft.block<3, 3>(6, 6) = Sophus::SO3d::hat(w_ie_n).matrix();;
    Ft.block<3, 3>(6, 9) = -C_nb;
    // B Matrix
    Bt.block<3, 3>(3, 3) = C_nb;
    Bt.block<3, 3>(6, 0) = -C_nb;
}

void UpdateCovariance(Eigen::Matrix<double, 15, 12> &Bt, 
                            Eigen::Matrix<double, 15, 1> &X_, 
                            Eigen::Matrix<double, 15, 15> &P_, 
                            const double T)
{
    Ft = Eigen::Matrix<double, 15, 15>::Identity() + Ft * T;
    Bt = Bt * T;

    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
    Q.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity() * gyro_noise * gyro_noise;
    Q.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * acc_noise * acc_noise;
    Q.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity() * gyro_bias_noise * gyro_bias_noise;
    Q.block<3, 3>(9, 9) = Eigen::Matrix<double, 3, 3>::Identity() * acc_bias_noise * acc_bias_noise;

    // only update error covariance P_, given that the X_ is zero
    P_ = Ft * P_ * Ft.transpose() + Bt * Q * Bt.transpose();
}

void HandleGNSSData(Eigen::Vector3d &gnss_xyz)
{
    double geo_x, geo_y, geo_z;
    local_geo_transformer.Forward(current_gnss_data_.lla(0), current_gnss_data_.lla(1), current_gnss_data_.lla(2), 
                                  geo_x, geo_y, geo_z);
    gnss_xyz = Eigen::Vector3d(geo_x, geo_y, geo_z);

    if (enabled_one_stage_spoofing)
    {
        addOneStageGNSSOffset(gnss_xyz);
    }
    if (enabled_two_stage_spoofing)
    {
        addTwoStageGNSSOffset(current_pose, current_lidar_data_, gnss_xyz);
    }
}

bool Correct(Eigen::Vector3d &gnss_xyz, 
             const std::string &sub_motion_state)
{

    if (fusion_strategy == "gnss")
    {
        return HandleGnssStrategy(gnss_xyz, sub_motion_state, fusion_strategy);
    }
    else if (fusion_strategy == "gnss_odom")
    {
        return HandleGnssOdomStrategy(gnss_xyz, sub_motion_state, fusion_strategy);
    }
    else if (fusion_strategy == "lidar")
    {
        return HandleLidarStrategy(gnss_xyz, sub_motion_state, fusion_strategy);
    }
    else if (fusion_strategy == "gnss_lidar")
    {
        return HandleGnssLidarStrategy(gnss_xyz, sub_motion_state, fusion_strategy);
    }
    else if (fusion_strategy == "gnss_odom_lidar")
    {
        return HandleGnssOdomLidarStrategy(gnss_xyz, sub_motion_state, fusion_strategy);
    }
    else
    {
        std::cout << "Unknown fusion strategy: " << fusion_strategy << std::endl;
        return false;
    }
}

bool HandleGnssStrategy(Eigen::Vector3d &gnss_xyz, 
                         const std::string &sub_motion_state, 
                         const std::string &fusion_strategy)
{
    if (fusion_strategy != "gnss")
    {
        return true;
    }

    Eigen::MatrixXd CPosi, RPosi;
    int degrees_of_freedom = 3;
    double chi_squared_critical = 7.815; // Corresponding to 3 degrees of freedom and 95% confidence level

    Eigen::Vector3d dp_gnss = current_pose.state.p - gnss_xyz;
    YPosi.block<3, 1>(0, 0) = dp_gnss;

    GPosi = Eigen::Matrix<double, 3, 15>::Zero();
    GPosi.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    CPosi = Eigen::Matrix<double, 3, 3>::Identity();

    RPosi = Eigen::Matrix3d::Identity() * gnss_posi_nosie * gnss_posi_nosie;

    if (!ChiSquaredTest(YPosi, GPosi, current_error_state.P_, RPosi, degrees_of_freedom, chi_squared_critical))
    {
        std::cout << "GNSS observation rejected in gnss model" << std::endl;
        return false;
    }

    UpdateKalmanGain(YPosi, GPosi, CPosi, RPosi, sub_motion_state);

    return true;
}


bool HandleLidarStrategy(Eigen::Vector3d &gnss_xyz, 
                          const std::string &sub_motion_state, 
                          const std::string &fusion_strategy)
{
    if (fusion_strategy != "lidar")
    {
        return true;
    }

    Eigen::MatrixXd CPose, RPose;
    int degrees_of_freedom = 6;
    double chi_squared_critical = 12.592; // Corresponding to 6 degrees of freedom and 95% confidence level
    // get lidar position and rotation
    Eigen::Vector3d p_lidar = current_lidar_data_.p;
    Eigen::Quaterniond q_lidar = current_lidar_data_.q;
    // lidar position and rotation error
    Eigen::Vector3d dp_lidar = current_pose.state.p - p_lidar;
    Eigen::Matrix<double, 3, 3> delta_rot = current_pose.state.q.toRotationMatrix() * q_lidar.toRotationMatrix().transpose();
    delta_rot = delta_rot - Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Vector3d dq_lidar(delta_rot(1, 2), delta_rot(2, 0), delta_rot(0, 1));

    YPose.block<3, 1>(0, 0) = dp_lidar;
    YPose.block<3, 1>(3, 0) = dq_lidar; 

    GPose = Eigen::Matrix<double, 6, 15>::Zero();
    GPose.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    GPose.block<3, 3>(3, 6) = Eigen::Matrix<double, 3, 3>::Identity();

    CPose = Eigen::Matrix<double, 6, 6>::Identity();

    RPose = Eigen::Matrix<double, 6, 6>::Identity();
    RPose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * lidar_posi_noise * lidar_posi_noise;
    RPose.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * lidar_ori_noise * lidar_ori_noise;

    if (!ChiSquaredTest(YPose, GPose, current_error_state.P_, RPose, degrees_of_freedom, chi_squared_critical))
    {
        std::cout << "Lidar observation rejected." << std::endl;
        return false;
    }

    UpdateKalmanGain(YPose, GPose, CPose, RPose, sub_motion_state);

    return true;
}

bool HandleGnssOdomStrategy(Eigen::Vector3d &gnss_xyz, 
                             const std::string &sub_motion_state, 
                             const std::string &fusion_strategy)
{
    if (fusion_strategy != "gnss_odom")
    {
        return true;
    }

    Eigen::MatrixXd CPosiVel, RPosiVel;
    int degrees_of_freedom = 6;
    double chi_squared_critical = 12.592; // Corresponding to 6 degrees of freedom and 95% confidence level

    Eigen::Matrix3d C_bn = current_pose.state.q.toRotationMatrix().transpose();
    Eigen::Matrix3d vnx = Sophus::SO3d::hat(current_pose.state.v).matrix();

    Eigen::Vector3d dp_gnss = current_pose.state.p - gnss_xyz;
    Eigen::Vector3d dv_odo = C_bn * current_pose.state.v - Eigen::Vector3d(0, current_odo_data_.odo, 0);

    YPosiVel.block<3, 1>(0, 0) = dp_gnss;
    YPosiVel.block<3, 1>(3, 0) = dv_odo; 

    GPosiVel = Eigen::Matrix<double, 6, 15>::Zero();
    GPosiVel.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    GPosiVel.block<3, 3>(3, 3) = C_bn;
    GPosiVel.block<3, 3>(3, 6) = -C_bn * vnx;

    CPosiVel = Eigen::Matrix<double, 6, 6>::Identity();

    RPosiVel = Eigen::Matrix<double, 6, 6>::Identity();
    RPosiVel.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gnss_posi_nosie * gnss_posi_nosie;
    RPosiVel.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * odom_vel_noise * odom_vel_noise;

    if (!ChiSquaredTest(YPosiVel, GPosiVel, current_error_state.P_, RPosiVel, degrees_of_freedom, chi_squared_critical))
    {
        return false;
    }

    UpdateKalmanGain(YPosiVel, GPosiVel, CPosiVel, RPosiVel, sub_motion_state);

    return true;
}

bool HandleGnssLidarStrategy(Eigen::Vector3d &gnss_xyz, 
                              const std::string &sub_motion_state, 
                              const std::string &fusion_strategy)
{
    if (fusion_strategy != "gnss_lidar")
    {
        return true;
    }

    Eigen::MatrixXd CPosiPose, RPosiPose;
    int degrees_of_freedom = 9;
    double chi_squared_critical = 16.919; // Corresponding to 9 degrees of freedom and 95% confidence level
    Eigen::Matrix3d C_bn = current_pose.state.q.toRotationMatrix().transpose();
    // get lidar position and rotation
    Eigen::Vector3d p_lidar = current_lidar_data_.p;
    Eigen::Quaterniond q_lidar = current_lidar_data_.q;
    // gnss position error
    Eigen::Vector3d dp_gnss = current_pose.state.p - gnss_xyz;
    // lidar position and rotation error
    Eigen::Vector3d dp_lidar = current_pose.state.p - p_lidar;
    Eigen::Matrix<double, 3, 3> delta_rot = current_pose.state.q.toRotationMatrix() * q_lidar.toRotationMatrix().transpose();
    delta_rot = delta_rot - Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Vector3d dq_lidar(delta_rot(1, 2), delta_rot(2, 0), delta_rot(0, 1));
    
    YPosiPose.block<3, 1>(0, 0) = dp_lidar;
    YPosiPose.block<3, 1>(3, 0) = dq_lidar; 
    YPosiPose.block<3, 1>(6, 0) = dp_gnss; 

    GPosiPose = Eigen::Matrix<double, 9, 15>::Zero();
    GPosiPose.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    GPosiPose.block<3, 3>(3, 6) = Eigen::Matrix<double, 3, 3>::Identity();
    GPosiPose.block<3, 3>(6, 0) = Eigen::Matrix<double, 3, 3>::Identity();

    CPosiPose = Eigen::Matrix<double, 9, 9>::Identity();

    RPosiPose = Eigen::Matrix<double, 9, 9>::Identity();
    RPosiPose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * lidar_posi_noise * lidar_posi_noise;
    RPosiPose.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * lidar_ori_noise * lidar_ori_noise;
    RPosiPose.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * gnss_posi_nosie * gnss_posi_nosie;

    if (!ChiSquaredTest(YPosiPose, GPosiPose, current_error_state.P_, RPosiPose, degrees_of_freedom, chi_squared_critical))
    {
        std::cout << "Lidar+GNSS observation rejected." << std::endl;
        return false;
    }

    UpdateKalmanGain(YPosiPose, GPosiPose, CPosiPose, RPosiPose, sub_motion_state);

    return true;
}

bool HandleGnssOdomLidarStrategy(Eigen::Vector3d &gnss_xyz, 
                                  const std::string &sub_motion_state, 
                                  const std::string &fusion_strategy)
{
    if (fusion_strategy != "gnss_odom_lidar")
    {
        return true;
    }

    Eigen::MatrixXd CPosiPoseVel, RPosiPoseVel;
    int degrees_of_freedom = 12;
    double chi_squared_critical = 21.026; // Corresponding to 12 degrees of freedom and 95% confidence level

    Eigen::Matrix3d C_bn = current_pose.state.q.toRotationMatrix().transpose();
    Eigen::Matrix3d vnx = Sophus::SO3d::hat(current_pose.state.v).matrix();
    // get lidar position and rotation
    Eigen::Vector3d p_lidar = current_lidar_data_.p;
    Eigen::Quaterniond q_lidar = current_lidar_data_.q;
    // gnss position error
    Eigen::Vector3d dp_gnss = current_pose.state.p - gnss_xyz;
    // lidar position and rotation error
    Eigen::Vector3d dp_lidar = current_pose.state.p - p_lidar;
    Eigen::Matrix<double, 3, 3> delta_rot = current_pose.state.q.toRotationMatrix() * q_lidar.toRotationMatrix().transpose();
    delta_rot = delta_rot - Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Vector3d dq_lidar(delta_rot(1, 2), delta_rot(2, 0), delta_rot(0, 1));
    // odom velocity error
    Eigen::Vector3d dv_odo = C_bn * current_pose.state.v - Eigen::Vector3d(0, current_odo_data_.odo, 0);

    // add lidar_latest
    lidartemp.time = current_lidar_data_.time;
    lidartemp.position = current_lidar_data_.p;

    YPosiPoseVel.block<3, 1>(0, 0) = dp_lidar;
    YPosiPoseVel.block<3, 1>(3, 0) = dv_odo;
    YPosiPoseVel.block<3, 1>(6, 0) = dq_lidar;
    YPosiPoseVel.block<3, 1>(9, 0) = dp_gnss;

    GPosiPoseVel = Eigen::Matrix<double, 12, 15>::Zero();
    GPosiPoseVel.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    GPosiPoseVel.block<3, 3>(3, 3) = C_bn;
    GPosiPoseVel.block<3, 3>(3, 6) = -C_bn * vnx;
    GPosiPoseVel.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity();
    GPosiPoseVel.block<3, 3>(9, 0) = Eigen::Matrix<double, 3, 3>::Identity();

    CPosiPoseVel = Eigen::Matrix<double, 12, 12>::Identity();

    RPosiPoseVel = Eigen::Matrix<double, 12, 12>::Identity();
    RPosiPoseVel.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * lidar_posi_noise * lidar_posi_noise;
    RPosiPoseVel.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * odom_vel_noise * odom_vel_noise;
    RPosiPoseVel.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * lidar_ori_noise * lidar_ori_noise;
    RPosiPoseVel.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * gnss_posi_nosie * gnss_posi_nosie;

    if (!ChiSquaredTest(YPosiPoseVel, GPosiPoseVel, current_error_state.P_, RPosiPoseVel, degrees_of_freedom, chi_squared_critical))
    {
        std::cout << "Lidar+GNSS+Odom observation rejected." << std::endl;
        return false;
    }

    UpdateKalmanGain(YPosiPoseVel, GPosiPoseVel, CPosiPoseVel, RPosiPoseVel, sub_motion_state);

    return true;
}

void UpdateKalmanGain(Eigen::VectorXd &Y,
                      Eigen::MatrixXd &Gt,
                      Eigen::MatrixXd &Ct, 
                      Eigen::MatrixXd &R, 
                      const std::string &sub_motion_state)
{
    Eigen::MatrixXd K = current_error_state.P_ * Gt.transpose() * (Gt * current_error_state.P_ * Gt.transpose() + Ct * R * Ct.transpose()).inverse();

    utilityFunctions.SaveMatrixToFile(K, "K", sub_motion_state);
    utilityFunctions.SaveMatrixToFile(Gt, "G", sub_motion_state);
    utilityFunctions.SaveMatrixToFile(current_error_state.P_, "P", sub_motion_state);

    current_error_state.X_ = current_error_state.X_ + K * (Y - Gt * current_error_state.X_);
    current_error_state.P_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * Gt) * current_error_state.P_;

    EliminateError();
    ResetState();
}

void EliminateError()
{
    // Eliminate position and velocity error
    current_pose.state.p = current_pose.state.p - current_error_state.X_.block<3, 1>(0, 0);
    current_pose.state.v = current_pose.state.v - current_error_state.X_.block<3, 1>(3, 0);


    // Eliminate orientation error
    Eigen::Vector3d theta = current_error_state.X_.block<3, 1>(6, 0);
    double angle = theta.norm();
    Eigen::Quaterniond dq;
    if (angle > std::numeric_limits<double>::epsilon()) {
        Eigen::Vector3d axis = theta / angle;
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    } else {
        dq = Eigen::Quaterniond::Identity();
    }
    current_pose.state.q = dq * current_pose.state.q;
    current_pose.state.q.normalize();
    
    // Eliminate gyro and acc bias error
    current_pose.state.bg = current_pose.state.bg - current_error_state.X_.block<3, 1>(9, 0);
    current_pose.state.ba = current_pose.state.ba - current_error_state.X_.block<3, 1>(12, 0);

    // update spoof pose
    current_spoof_pose.time = current_pose.time;
    current_spoof_pose.p(0) = current_pose.state.p(0);
    current_spoof_pose.p(1) = current_pose.state.p(1);
    
}

void ResetState()
{
    // reset current state
    current_error_state.X_ = 
    Eigen::VectorXd::Zero(current_error_state.X_.size());

    msf_latest = current_pose.state.p.transpose();
    Eigen::Quaterniond q = current_pose.state.q;
    yaw_latest = utilityFunctions.QuaternionToYaw(q);

    if (std::abs(lidartemp.time  - current_pose.time <= 0.01))
    {
        lidar_latest = lidartemp.position.transpose();
    }

}

bool ChiSquaredTest(const Eigen::VectorXd &Y, 
                    const Eigen::MatrixXd &Gt, 
                    const Eigen::MatrixXd &P, 
                    const Eigen::MatrixXd &R, 
                    int degrees_of_freedom, 
                    double chi_squared_critical)
{
    if (!enable_chi_squared_test)
    {
        // LOG(INFO) << "Chi-Squared Test is disabled in yaml.";
        return true;
    }
    Eigen::VectorXd residual = Y - Gt * current_error_state.X_;
    Eigen::MatrixXd S = Gt * P * Gt.transpose() + R;
    double chi_squared = residual.transpose() * S.inverse() * residual;
    if (chi_squared > chi_squared_critical)
    {
        LOG(WARNING) << "Chi-Squared Test Failed: " << chi_squared << " > " << chi_squared_critical;
        LOG(INFO) << "Residual causing the failure: " << residual.transpose();
        LOG(INFO) << "Sub motion state causing the failure: " << sub_motion_state;

        return false;
    }
    return true;
}



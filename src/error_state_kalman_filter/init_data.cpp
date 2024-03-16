//init_data.cpp
#include "init_data.h"

POSEData current_pose;
ErrorState current_error_state;
GeographicLib::LocalCartesian local_geo_transformer(32, 120, 0);

// sensor noise
std::vector<double> init_dx;
std::vector<double> prior;

bool InitSensor()
{
    static bool gnss_inited = false;
    int popped_gnss_count = 0;

    if (!gnss_inited && !gnss_data_buff_.empty()) {
        GNSSData& gnss_data = gnss_data_buff_.front();

        local_geo_transformer.Reset(gnss_data.lla.x(), gnss_data.lla.y(), gnss_data.lla.z());

        gnss_inited = true;

        std::cout << "Init local map frame at: " << gnss_data.lla.x() << ", "
                  << gnss_data.lla.y() << ", " << gnss_data.lla.z()
                  << std::endl;
    }

    for (auto gnss_it = gnss_data_buff_.begin(); gnss_it != gnss_data_buff_.end();)
    {
        if (imu_data_buff_.empty() || imu_data_buff_.front().time <= gnss_it->time)
        {
            if (popped_gnss_count > 0)
            {
                std::cout << "Popped " << popped_gnss_count << " GNSS data points." << std::endl;
            }
            return true;
        }
        else
        {
            gnss_it = gnss_data_buff_.erase(gnss_it);
            popped_gnss_count++;
        }
    }

    if (popped_gnss_count > 0)
    {
        std::cout << "Popped " << popped_gnss_count << " GNSS data points." << std::endl;
    }
    return gnss_inited;
}

bool InitPose()
{
    static bool pose_inited = false;
    if (pose_inited)
    {
        return true;
    }

    if (!InitializePositionAndVelocity() || !InitializeAttitudeAndBias() || !InitializeErrorStateAndCov())
    {
        return false;
    }

    pose_inited = true;
    return true;
}

bool InitializePositionAndVelocity()
{
    Eigen::Vector3d init_pos_offset(init_dx[0], init_dx[0], init_dx[0]);
    Eigen::Vector3d init_vel_offset(init_dx[1], init_dx[1], init_dx[1]);

    current_pose.time = current_ref_data_.time;
    current_pose.state.p = current_ref_data_.state.p + init_pos_offset;
    current_pose.state.v = current_ref_data_.state.v + init_vel_offset;

    return true;
}

bool InitializeAttitudeAndBias()
{
    Eigen::Vector3d rotation_vector(init_dx[2], init_dx[2], init_dx[2]);
    double rotation_angle = rotation_vector.norm();

    if (rotation_angle > std::numeric_limits<double>::epsilon())
    {
        Eigen::Vector3d rotation_axis = rotation_vector.normalized();
        double cos_half_angle = std::cos(rotation_angle / 2);
        double sin_half_angle = std::sin(rotation_angle / 2);
        Eigen::Quaterniond rotation_delta(
            cos_half_angle,
            rotation_axis.x() * sin_half_angle,
            rotation_axis.y() * sin_half_angle,
            rotation_axis.z() * sin_half_angle);

        current_pose.state.q = current_ref_data_.state.q * rotation_delta;
        current_pose.state.q.normalize();
    }
    else
    {
        current_pose.state.q = current_ref_data_.state.q;
    }

    if (std::abs(current_pose.state.q.norm() - 1.0) > std::numeric_limits<double>::epsilon())
    {
        std::cerr << "Error: Attitude initialization resulted in an invalid quaternion." << std::endl;
        return false;
    }
    
    Eigen::Vector3d gyro_bias(init_dx[3], init_dx[3], init_dx[3]);
    Eigen::Vector3d accel_bias(init_dx[4], init_dx[4], init_dx[4]);

    current_pose.state.bg = current_ref_data_.state.bg + gyro_bias;
    current_pose.state.ba = current_ref_data_.state.ba + accel_bias;

    return true;
}

bool InitializeErrorStateAndCov()
{
    current_error_state.X_ = Eigen::Matrix<double, 15, 1>::Zero();
    current_error_state.P_ = Eigen::Matrix<double, 15, 15>::Zero();
    current_error_state.P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * prior[0];
    current_error_state.P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * prior[1];
    current_error_state.P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * prior[2];
    current_error_state.P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * prior[3];
    current_error_state.P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * prior[4];

    return true;
}

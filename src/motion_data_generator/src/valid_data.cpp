//valid_data.cpp
#include "valid_data.h"

// current sensor data
std::deque<IMUData> current_imu_data_;
GNSSData current_gnss_data_;
ODOData current_odo_data_;
POSEData current_ref_data_;
LIDARData current_lidar_data_;

// lidar noise
std::string lidar_noise_level;
double lidar_noise_std_dev;
const double no_error_std_dev = 0.0;
const double high_accuracy_std_dev = 0.01;
const double mid_accuracy_std_dev = 0.05;
const double low_accuracy_std_dev = 0.1;

bool Initialization()
{

    if (enable_calculateFrequency)
    {
        utilityFunctions.CalculateFrequency();
    }
    if (!ValidData(false))
    {
        std::cerr << "Error validing data in Initialization" << std::endl;
        return false;
    }
    if (!InitSensor())
    {
        std::cerr << "InitSensor Error!" << std::endl;
        return false;
    }

    if (!InitPose())
    {
        std::cerr << "InitPose Error!" << std::endl;
        return false;
    }

    utilityFunctions.SaveData();
    return true;
}

bool ValidData(bool inited)
{
    double valid_time;
    static int ref_data_counter = 0;

    if (!ValidGNSSData(valid_time))
    {
        return false;
    }

    if (!ValidIMUData(valid_time, inited))
    {
        return false;
    }

    if (!ValidODOData(valid_time))
    {
        return false;
    }

    if (!ValidRefData(valid_time, ref_data_counter))
    {
        return false;
    }

    if (!ValidLidarData(valid_time, current_ref_data_, ref_data_counter))
    {
        ref_data_counter = 0;
    }
    else
    {
        ref_data_counter++;
    }

    return true;
}

bool ValidGNSSData(double &valid_time)
{
    if (gnss_data_buff_.empty())
    {
        return false;
    }
    current_gnss_data_ = gnss_data_buff_.front();
    valid_time = current_gnss_data_.time;
    return true;
}

bool ValidIMUData(const double &valid_time, 
                 bool inited)
{
    if (imu_data_buff_.size() > 1)
    {
        while (!inited && imu_data_buff_[1].time < valid_time)
        {
            imu_data_buff_.pop_front();
        }

        if (!inited)
        {
            current_imu_data_.clear();
            IMUData front_data = imu_data_buff_.at(0);
            IMUData back_data = imu_data_buff_.at(1);
            double front_scale = (back_data.time - valid_time) / (back_data.time - front_data.time);
            double back_scale = (valid_time - front_data.time) / (back_data.time - front_data.time);
            IMUData valided_data;
            valided_data.time = valid_time;
            valided_data.acc = utilityFunctions.LinearInterpolate(front_data.acc, back_data.acc, front_scale, back_scale);
            valided_data.gyro = utilityFunctions.LinearInterpolate(front_data.gyro, back_data.gyro, front_scale, back_scale);
            current_imu_data_.push_back(valided_data);
            imu_data_buff_.pop_front();
            gnss_data_buff_.pop_front();
        }
        else
        {
            if (imu_data_buff_.back().time < valid_time)
            {
                return false;
            }
            while (current_imu_data_.size() > 1)
            {
                current_imu_data_.pop_front();
            }
            while (imu_data_buff_.front().time < valid_time)
            {
                current_imu_data_.push_back(imu_data_buff_.front());
                imu_data_buff_.pop_front();
            }
            IMUData valided_data = current_imu_data_.back();
            current_imu_data_.push_back(valided_data);
            gnss_data_buff_.pop_front();
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool ValidODOData(const double &valid_time)
{
    if (odo_data_buff_.size() > 1)
    {
        while (odo_data_buff_[1].time < valid_time)
        {
            odo_data_buff_.pop_front();
        }

        if (odo_data_buff_.size() > 1)
        {
            ODOData front_data = odo_data_buff_.at(0);
            ODOData back_data = odo_data_buff_.at(1);
            double front_scale = (back_data.time - valid_time) / (back_data.time - front_data.time);
            double back_scale = (valid_time - front_data.time) / (back_data.time - front_data.time);
            current_odo_data_.time = valid_time;
            current_odo_data_.odo = utilityFunctions.LinearInterpolate(front_data.odo, back_data.odo, front_scale, back_scale);
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool ValidRefData(const double &valid_time, 
                int &counter)
{
    if (ref_data_buff_.size() > 1)
    {
        while (ref_data_buff_[1].time < valid_time)
        {
            ref_data_buff_.pop_front();
            counter++;
        }

        if (ref_data_buff_.size() > 1)
        {
            POSEData front_data = ref_data_buff_.at(0);
            POSEData back_data = ref_data_buff_.at(1);
            double front_scale = (back_data.time - valid_time) / (back_data.time - front_data.time);
            double back_scale = (valid_time - front_data.time) / (back_data.time - front_data.time);

            current_ref_data_.time = valid_time;
            current_ref_data_.state.p = utilityFunctions.LinearInterpolate(front_data.state.p, back_data.state.p, front_scale, back_scale);
            current_ref_data_.state.v = utilityFunctions.LinearInterpolate(front_data.state.v, back_data.state.v, front_scale, back_scale);
            current_ref_data_.state.q = front_data.state.q.slerp(back_scale, back_data.state.q);
            current_ref_data_.state.bg = utilityFunctions.LinearInterpolate(front_data.state.bg, back_data.state.bg, front_scale, back_scale);
            current_ref_data_.state.ba = utilityFunctions.LinearInterpolate(front_data.state.ba, back_data.state.ba, front_scale, back_scale);
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool ValidLidarData(const double &valid_time, 
                   const POSEData &current_ref_data_, 
                   const int counter)
{
    const int lidar_counter = 10;

    if (counter >= lidar_counter)
    {
        current_lidar_data_.time = valid_time;
        current_lidar_data_.p = current_ref_data_.state.p;
        current_lidar_data_.q = current_ref_data_.state.q;
        AddLidarNoise(current_lidar_data_.p, current_lidar_data_.q, lidar_noise_std_dev);

        return true;
    }

    return false;
}

void AddLidarNoise(Eigen::Vector3d &position, 
                   Eigen::Quaterniond &orientation, 
                   double std_dev)
{
    if (std_dev > 0.0)
    {
        std::random_device rd;
        std::default_random_engine generator(rd());
        std::normal_distribution<double> distribution(0.0, std_dev);

        // position noise
        Eigen::Vector3d position_noise(distribution(generator), distribution(generator), distribution(generator));
        position += position_noise;

        // orientation noise
        double angle_noise = distribution(generator);
        Eigen::Quaterniond rotation_noise;
        rotation_noise = Eigen::AngleAxisd(angle_noise, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(angle_noise, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(angle_noise, Eigen::Vector3d::UnitZ());
        orientation = orientation * rotation_noise;
    }
}

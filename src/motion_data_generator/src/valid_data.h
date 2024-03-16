//valid_data.h
#ifndef VALID_DATA_H
#define VALID_DATA_H

#include "../../app/utilityFunctions.h"

// --------------------------  2.valid data, init sensor and init pose -------------------------- //
// current sensor data
extern std::deque<IMUData> current_imu_data_;
extern GNSSData current_gnss_data_;
extern ODOData current_odo_data_;
extern POSEData current_ref_data_;
extern LIDARData current_lidar_data_;

// lidar noise
extern std::string lidar_noise_level;
extern double lidar_noise_std_dev;
extern const double no_error_std_dev;
extern const double high_accuracy_std_dev;
extern const double mid_accuracy_std_dev;
extern const double low_accuracy_std_dev;

bool ValidData(bool inited);
bool ValidIMUData(const double &valid_time,
                 bool inited);
bool ValidODOData(const double &valid_time);
bool ValidLidarData(const double &valid_time,
                   const POSEData &current_ref_data_,
                   const int counter);
void AddLidarNoise(Eigen::Vector3d &position,
                   Eigen::Quaterniond &orientation,
                   double std_dev);
bool ValidGNSSData(double &valid_time);
bool ValidRefData(const double &valid_time,
                int &counter);

#endif // VALID_DATA_H
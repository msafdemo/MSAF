//utilityFunctions.h
#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H
#include <deque>
#include <cmath>
#include <vector>
#include <random>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <sys/stat.h>
#include <sophus/so3.hpp> 
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include "Geocentric/LocalCartesian.hpp"

#include "dataStructures.h"
#include "../motion_data_generator/src/load_data.h"
#include "../motion_data_generator/src/valid_data.h"
#include "../error_state_kalman_filter/init_data.h"
#include "../error_state_kalman_filter/error_state_kalman_filter.h"
#include "../state_dependency_analyzer/state_dependency_analyzer.h"
#include "../injector/injector.h"
class UtilityFunctions {
    
public:

    UtilityFunctions() {}

    Eigen::Vector3d QuaternionToEulerAngles(const Eigen::Quaterniond &q);
    double QuaternionToYaw(const Eigen::Quaterniond &q);
    double getDistance(const Eigen::Vector3d &point1,
                const Eigen::Vector3d &point2);
    void SaveMatrixToFile(const Eigen::MatrixXd &matrix,
                    const std::string &prefix,
                    const std::string &sub_motion_state);
    bool checkPathExists(const std::string &path,
                    const std::string &errorMessage);
    bool SavePose(std::ofstream &save_points, 
              POSEData &pose);
    bool SavePoseAsKITTI(std::ofstream &save_points, 
                                       POSEData &pose);
    bool SavePosi(std::ofstream &save_points, 
              POSEData &pose);
    bool SaveData();
    void CalculateFrequency();
    void CalculateSensorFrequency(const std::vector<double> &time_stamps,
                                const std::string &data_name);
    bool SaveSpoofPose(std::ofstream &save_points,
                       SpoofData &pose);    
    template <typename T> 
    T LinearInterpolate(const T &start, 
                        const T &end, 
                        double start_scale, 
                        double end_scale)
    {
        return start * start_scale + end * end_scale;
    }
};

#endif // UTILITY_FUNCTIONS_H
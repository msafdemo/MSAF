//dataStructures.h
#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <Eigen/Geometry>
#include "Geocentric/LocalCartesian.hpp"
#include <deque>
#include <random>
#include <yaml-cpp/yaml.h>
#include <cstdio>
#include <glog/logging.h>
#include <cmath>

struct State
{
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
};
struct ErrorState
{
    Eigen::Matrix<double, 15, 1> X_;
    Eigen::Matrix<double, 15, 15> P_;
};
struct IMUData
{
    double time;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};
struct GNSSData
{
    double time;
    Eigen::Vector3d lla;
    Eigen::Vector3d v;
};
struct POSEData
{
    double time;
    State state;
};
struct LIDARData
{
    double time;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
};
struct ODOData
{
    double time;
    double odo;
};

struct SpoofData
{
    double time;
    Eigen::Vector3d p;
};


#endif // DATA_STRUCTURES_H
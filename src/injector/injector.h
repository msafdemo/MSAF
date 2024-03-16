//Spoofing.h
#ifndef INJECTOR_H
#define INJECTOR_H

#include "../app/utilityFunctions.h"

// one stage injection
extern double enabled_one_stage_spoofing;
extern double spoofing_time;
extern int spoofing_count;
extern double spoofing_dev_straight;
extern double spoofing_dev_turning;
extern std::vector<double> injectionPoints;
extern int dev_count;
extern int nextInjectionIndex;
extern double lastX;

// two stage injection
extern YAML::Node TwoStageSpoofingConfig;
extern bool enable_chi_squared_test;
extern Eigen::Vector3d msf_latest;
extern Eigen::Vector3d lidar_latest;
extern double yaw_latest;
extern double enabled_two_stage_spoofing;
extern std::string spoofing_method;
extern double speed_threshold;
extern double start_time;
extern double end_time;
extern double d;
extern double f;
extern double lowerbound;
extern double upperbound;

bool LoadOneStageSpoofingConfig(const YAML::Node &root_yaml_node);
bool LoadTwoStageSpoofingConfig();
bool addOneStageGNSSOffset(Eigen::Vector3d &gnss_xyz);
bool addTwoStageGNSSOffset(const POSEData &current_pose,
                           const LIDARData &current_lidar_data_,
                           Eigen::Vector3d &gnss_xyz);
void addOneStageGNSSOffsetForStraight(Eigen::Vector3d &gnss_xyz);
bool addOneStageGNSSOffsetForTurning(Eigen::Vector3d &gnss_xyz);


#endif // INJECTOR_H
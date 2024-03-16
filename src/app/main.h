//main.h
#ifndef MAIN_H
#define MAIN_H
#include "utilityFunctions.h"

// LoadConfigAndData
extern YAML::Node root_yaml_node;
extern YAML::Node motion_state_yaml_node;

extern std::string fusion_strategy;

// calculate frequency
extern bool enable_calculateFrequency;
extern int predictCounter;
extern int correctCounter;

// output file stream
extern std::ofstream ref_data_ofs_;
extern std::ofstream ref_data_posi_ofs_;
extern std::ofstream eskf_data_ofs;
extern std::ofstream spoof_pose_ofs;

int main(int argc, char *argv[]);

bool ParseArguments(int argc, char *argv[],
                    std::string &motion_state_suffix,
                    std::string &folder_suffix,
                    std::string &obs_result_path);

bool Initialization();
bool LoadConfigAndData();
bool LoadMotionStateConfig(const YAML::Node &motion_state_yaml_node,
                           const YAML::Node &root_yaml_node,
                           const std::string &motion_state_yaml_path);
bool InitializeAndLoadData(const YAML::Node &root_yaml_node,
                                  const std::string &sim_data_base_path);
bool LoadSensorNoiseAndFusionConfig(const YAML::Node &root_yaml_node);

#endif // MAIN_H
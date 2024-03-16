//main.cpp
#include "main.h"

YAML::Node root_yaml_node;
YAML::Node motion_state_yaml_node;

// calculate frequency
bool enable_calculateFrequency;
int predictCounter;
int correctCounter;

std::string fusion_strategy = " ";

int main(int argc, char *argv[])
{
    std::string folder_suffix, motion_state_suffix, obs_result_path;

    if (!ParseArguments(argc, argv, folder_suffix, motion_state_suffix, obs_result_path))
    {
        std::cout << "ParseArguments Error!" << std::endl;
        return 1;
    }
    
    if (!LoadConfigAndData())
    {
        std::cerr << "LoadConfigAndData Error!" << std::endl;
        return false;
    }

    if (!Initialization())
    {
        std::cout << "Initialization Error!" << std::endl;
        return -1;
    }

    int counter_for_while = 0;
    ExecuteFilterAndSaveData(counter_for_while);

    std::cout << "counter_for_while: " << counter_for_while << std::endl;
    std::cout << "predictCounter: " << predictCounter << std::endl;
    std::cout << "correctCounter : " << correctCounter << std::endl;

    int ysize = ComputeYSize(fusion_strategy);

    PerformSVDAndObservability(ysize, obs_result_path);

    return 0;
}

bool ParseArguments(int argc, char *argv[], std::string &motion_state_suffix, std::string &folder_suffix, std::string &obs_result_path)
{
    if (argc >= 2)
    {
        motion_state_suffix = argv[1];
        std::cout << "Argument 1 (motion_state_suffix): " << motion_state_suffix << std::endl;
    }
    else
    {
        std::cerr << "Missing argument 1 (motion_state_suffix)." << std::endl;
        return false;
    }

    if (argc >= 3 && argv[2][0] != '\0')
    {
        folder_suffix = argv[2];
        std::cout << "Argument 2 (folder_suffix): " << folder_suffix << std::endl;
    }
    else
    {
        // It's okay if folder_suffix is empty, handle accordingly
        std::cout << "Argument 2 (folder_suffix) is empty or not provided." << std::endl;
    }

    if (argc >= 4)
    {
        obs_result_path = argv[3];
        std::cout << "Argument 3 (obs_result_path): " << obs_result_path << std::endl;
    }
    else
    {
        obs_result_path = "data/obs_result/obs_result_" + motion_state_suffix + ".txt";
        std::cout << "Default for Argument 3 (obs_result_path): " << obs_result_path << std::endl;
    }

    return true;
}


bool LoadConfigAndData()
{
    std::string root_yaml_path = "config/config.yaml";
    std::string motion_state_yaml_path = "config/motion_config.yaml";

    if (!utilityFunctions.checkPathExists(root_yaml_path, "root_yaml_path") ||
        !utilityFunctions.checkPathExists(motion_state_yaml_path, "motion_state_yaml_path") ||
        !utilityFunctions.checkPathExists("data/sim_data/", "sim_data_base_path"))
    {
        return false;
    }

    root_yaml_node = YAML::LoadFile(root_yaml_path);
    motion_state_yaml_node = YAML::LoadFile(motion_state_yaml_path);

    if (!LoadMotionStateConfig(motion_state_yaml_node, root_yaml_node, motion_state_yaml_path))
    {
        return false;
    }

    motion_state = root_yaml_node["motion_state"].as<std::string>();
    std::string sim_data_base_path = "data/sim_data/" + motion_state + "/";

    if (!InitializeAndLoadData(root_yaml_node, sim_data_base_path))
    {
        return false;
    }

    if (!LoadSensorNoiseAndFusionConfig(root_yaml_node))
    {
        return false;
    }

    if (!LoadOneStageSpoofingConfig(root_yaml_node))
    {
        return false;
    }

    return true;
}

bool LoadMotionStateConfig(const YAML::Node &motion_state_yaml_node,
                           const YAML::Node &root_yaml_node,
                           const std::string &motion_state_yaml_path)
{
    if (motion_state_yaml_node["motion_state"] && motion_state_yaml_node["motion_state"].IsScalar())
    {
        motion_state = root_yaml_node["motion_state"].as<std::string>();
        sub_motion_state = motion_state_yaml_node["motion_state"].as<std::string>();
    }
    else
    {
        std::cerr << "sub_motion_state is not available or not a scalar in " << motion_state_yaml_path << std::endl;
        return false;
    }

    return true;
}

bool InitializeAndLoadData(const YAML::Node &root_yaml_node, 
                                  const std::string &sim_data_base_path)
{
    std::string full_path = sim_data_base_path + sub_motion_state + folder_suffix + "/";

    std::vector<std::pair<std::string, std::ofstream *>> fileMappings = {
        {"ref_data.txt", &ref_data_ofs_},
        {"ref_data_posi.txt", &ref_data_posi_ofs_},
        {"eskf_data.txt", &eskf_data_ofs},
        {"spoof_pose.txt", &spoof_pose_ofs}};

    for (const auto &fileMapping : fileMappings)
    {
        fileMapping.second->open(full_path + fileMapping.first, std::fstream::out);
        if (!fileMapping.second->is_open())
        {
            std::cerr << "Failed to open file: " << full_path + fileMapping.first << std::endl;
            return false;
        }
    }

    std::vector<std::string> dataPaths = {
        "time.csv",
        "accel-0.csv",
        "gyro-0.csv",
        "odo-0.csv",
        "ref_pos.csv",
        "ref_vel.csv",
        "ref_att_quat.csv",
        "gps_time.csv",
        "gps-0.csv"};

    for (auto &path : dataPaths)
    {
        path = full_path + path;
    }

    LoadData(dataPaths);

    return true;
}

bool LoadSensorNoiseAndFusionConfig(const YAML::Node &root_yaml_node)
{
    prior = root_yaml_node["prior"].as<std::vector<double>>();
    init_dx = root_yaml_node["init_dx"].as<std::vector<double>>();
    // process noise
    acc_noise = root_yaml_node["acc_noise"].as<double>();
    gyro_noise = root_yaml_node["gyro_noise"].as<double>();
    acc_bias_noise = root_yaml_node["acc_bias_noise"].as<double>();
    gyro_bias_noise = root_yaml_node["gyro_bias_noise"].as<double>();
    // measurement noise
    gnss_posi_nosie = root_yaml_node["gnss_posi_nosie"].as<double>();
    lidar_posi_noise = root_yaml_node["lidar_posi_noise"].as<double>();
    odom_vel_noise = root_yaml_node["odom_vel_noise"].as<double>();
    lidar_ori_noise = root_yaml_node["lidar_ori_noise"].as<double>();

    fusion_strategy = root_yaml_node["fusion_strategy"].as<std::string>();
    enable_calculateFrequency = root_yaml_node["calculateFrequency"].as<double>();
    enable_chi_squared_test = root_yaml_node["enable_chi_squared_test"].as<bool>();

    // lidar noise
    lidar_noise_level = root_yaml_node["sensor_noise_levels"]["lidar_noise"].as<std::string>();
    if (lidar_noise_level == "no_error")
        lidar_noise_std_dev = no_error_std_dev;
    else if (lidar_noise_level == "high_accuracy")
        lidar_noise_std_dev = high_accuracy_std_dev;
    else if (lidar_noise_level == "mid_accuracy")
        lidar_noise_std_dev = mid_accuracy_std_dev;
    else if (lidar_noise_level == "low_accuracy")
        lidar_noise_std_dev = low_accuracy_std_dev;
    else
        lidar_noise_std_dev = no_error_std_dev;
    return true;
}

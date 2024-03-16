//Spoofing.cpp
#include "injector.h"

// one stage injection
double enabled_one_stage_spoofing = 0.0;
double spoofing_time = 0.0;
int spoofing_count = 0;
double spoofing_dev_straight = 0.0;
double spoofing_dev_turning = 0.0;
std::vector<double> injectionPoints = {10.0, 19.0, 10.0};
int dev_count;
int nextInjectionIndex = 0;
double lastX = -1000.0;

// two stage injection
YAML::Node TwoStageSpoofingConfig;
bool enable_chi_squared_test = false;
Eigen::Vector3d msf_latest = Eigen::Vector3d::Zero();
Eigen::Vector3d lidar_latest = Eigen::Vector3d::Zero();
double yaw_latest = 0;
double enabled_two_stage_spoofing = 0.0;
std::string spoofing_method;
double speed_threshold = 0.0;
double start_time = 0.0;
double end_time = 0.0;
double d = 0.1;
double f = 0.0;
double lowerbound = 0.0;
double upperbound = 0.0;

bool LoadOneStageSpoofingConfig(const YAML::Node &root_yaml_node)
{
    spoofing_time = root_yaml_node["spoofing_time"].as<double>();
    spoofing_count = root_yaml_node["spoofing_count"].as<int>();
    spoofing_dev_straight = root_yaml_node["spoofing_dev_straight"].as<double>();
    spoofing_dev_turning = root_yaml_node["spoofing_dev_turning"].as<double>();
    enabled_one_stage_spoofing = root_yaml_node["enabled_one_stage_spoofing"].as<double>();

    enabled_two_stage_spoofing = root_yaml_node["enabled_two_stage_spoofing"].as<double>();

    if (motion_state == "turning_yaw")
    {
        injectionPoints = root_yaml_node["injectionPoints_for_yaw"].as<std::vector<double>>();
    }
    else if (motion_state == "turning_yaw_vel")
    {
        injectionPoints = root_yaml_node["injectionPoints_for_yaw_vel"].as<std::vector<double>>();
    }
    else
    {
        injectionPoints = std::vector<double>{0};
    }
    return true;
}

bool LoadTwoStageSpoofingConfig()
{
    try
    {
        TwoStageSpoofingConfig = YAML::LoadFile("config/config.yaml");
        start_time = TwoStageSpoofingConfig["two_stage_spoofing_time"]["start"].as<double>();
        end_time = TwoStageSpoofingConfig["two_stage_spoofing_time"]["end"].as<double>();
        d = TwoStageSpoofingConfig["spoofing_dev"]["d"].as<double>();
        f = TwoStageSpoofingConfig["spoofing_dev"]["f"].as<double>();
        lowerbound = TwoStageSpoofingConfig["spoofing_dev"]["lowerbound"].as<double>();
        upperbound = TwoStageSpoofingConfig["spoofing_dev"]["upperbound"].as<double>();
        spoofing_method = TwoStageSpoofingConfig["spoofing_method"].as<std::string>();
        speed_threshold = TwoStageSpoofingConfig["speed_threshold"].as<double>();
        return true;
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "Error in loading config file: " << e.what() << std::endl;
        return false;
    }
}

void addOneStageGNSSOffsetForStraight(Eigen::Vector3d &gnss_xyz)
{
    dev_count++;
    if (dev_count <= spoofing_count && dev_count > 0)
    {
        gnss_xyz[0] += spoofing_dev_straight;
        // std::cout << "Debug: spoofing in straight, add deviation: " << spoofing_dev_straight << " m\n"  << std::endl;
    }
}

bool addOneStageGNSSOffsetForTurning(Eigen::Vector3d &gnss_xyz)
{
    dev_count++;
    Eigen::Quaterniond q = current_pose.state.q;
    double theta = utilityFunctions.QuaternionToYaw(q);

    if (dev_count <= spoofing_count && dev_count > 0)
    {
        double temp_x = spoofing_dev_turning * cos(theta);
        double temp_y = spoofing_dev_turning * sin(theta);

        gnss_xyz[0] -= temp_x;
        gnss_xyz[1] -= temp_y;

        std::cout << "Debug: spoofing in turning, add deviation: " << temp_x << " m in x direction and " << temp_y << " m in y direction.\n"
                  << std::endl;
    }
    return true;
}

bool addOneStageGNSSOffset(Eigen::Vector3d &gnss_xyz)
{
    double currentCoordinate = 0.0;
    if (motion_state == "turning_yaw")
    {
        currentCoordinate = current_pose.state.p(0); // use currentX
    }
    else if (motion_state == "turning_yaw_vel")
    {
        currentCoordinate = current_pose.state.p(1); // use currentY
    }

    bool is_driving_straight = std::abs(current_imu_data_.back().gyro.norm()) < 1e-2;

    if (is_driving_straight && (current_pose.time >= spoofing_time))
    {
        addOneStageGNSSOffsetForStraight(gnss_xyz);
    }

    if (nextInjectionIndex < injectionPoints.size())
    {
        if ((lastX < injectionPoints[nextInjectionIndex] && currentCoordinate >= injectionPoints[nextInjectionIndex]) ||
            (lastX > injectionPoints[nextInjectionIndex] && currentCoordinate <= injectionPoints[nextInjectionIndex]))
        {
            if (!is_driving_straight) 
            {
                addOneStageGNSSOffsetForTurning(gnss_xyz);
            }
            nextInjectionIndex++; 
        }
    }
    lastX = currentCoordinate; // update lastX for next iteration
    return true;
}

bool addTwoStageGNSSOffset(const POSEData &current_pose, 
                           const LIDARData &current_lidar_data_, 
                           Eigen::Vector3d &gnss_xyz)
{
    if (!LoadTwoStageSpoofingConfig())
    {
        std::cerr << "Failed to load spoofing configuration." << std::endl;
        return false;
    }

    static int spoofing_count = 0;
    static int lowerbound_count = 0;
    double dis = utilityFunctions.getDistance(msf_latest, lidar_latest);
    Eigen::Vector3d current_velocity = current_pose.state.v;
    double current_speed = current_velocity.norm();

    bool is_within_spoofing_time = current_pose.time > start_time && current_pose.time <= end_time;

    if (is_within_spoofing_time)
    {
        bool should_spoof = false;
        if (spoofing_method == "speed")
        {
            should_spoof = current_speed > speed_threshold;
        }
        else if (spoofing_method == "distance")
        {
            if (dis > lowerbound)
            {
                should_spoof = true;
            }
        }

        if (should_spoof)
        {
            lowerbound_count++;
            if (lowerbound_count >= 50) // make sure kalman filter has converged
            {
                d = d * pow(f, spoofing_count);
                std::cout << "d: " << d << std::endl;
                spoofing_count++;
            }
        }
        else
        {
            spoofing_count = 0;
            lowerbound_count = 0;
            d = 0;
        }
    }
    else
    {
        spoofing_count = 0;
        lowerbound_count = 0;
        d = 0;
    }

    gnss_xyz[0] += d;
    static int print_count = 0;
    print_count++;
    if (print_count >= 10) {
        printf("time: %.2lf,  dis: %.2lf, d: %.2lf, speed: %.2lf\n", current_pose.time, dis, d, current_speed);
        print_count = 0;
    }

    return true;
}

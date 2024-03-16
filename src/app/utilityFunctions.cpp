//utilityFunctions.cpp
# include "utilityFunctions.h"

// sensor data
std::ofstream ref_data_ofs_;
std::ofstream ref_data_posi_ofs_;
std::ofstream eskf_data_ofs;
std::ofstream spoof_pose_ofs;

double UtilityFunctions::QuaternionToYaw(const Eigen::Quaterniond &q)
{
    Eigen::Quaterniond normalized_q = q.normalized();
    double siny_cosp = 2.0 * (normalized_q.w() * normalized_q.z() + normalized_q.x() * normalized_q.y());
    double cosy_cosp = 1.0 - 2.0 * (normalized_q.y() * normalized_q.y() + normalized_q.z() * normalized_q.z());

    if (std::abs(cosy_cosp) < std::numeric_limits<double>::epsilon())
    {
        return std::atan2(2.0 * normalized_q.x() * normalized_q.w() - 2.0 * normalized_q.y() * normalized_q.z(),
                          1.0 - 2.0 * normalized_q.x() * normalized_q.x() - 2.0 * normalized_q.z() * normalized_q.z());
    }

    return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Vector3d UtilityFunctions::QuaternionToEulerAngles(const Eigen::Quaterniond &q)
{
    Eigen::Vector3d euler_angles;

    // Roll
    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    euler_angles(2) = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch
    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
    {
        euler_angles(1) = std::copysign(M_PI / 2, sinp);
    }
    else
    {
        euler_angles(1) = std::asin(sinp);
    }

    // Yaw
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    euler_angles(0) = std::atan2(siny_cosp, cosy_cosp);

    return euler_angles;
}

double UtilityFunctions::getDistance(const Eigen::Vector3d &point1, 
                   const Eigen::Vector3d &point2)
{
    return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
}

void UtilityFunctions::SaveMatrixToFile(const Eigen::MatrixXd &matrix, 
                      const std::string &prefix, 
                      const std::string &sub_motion_state)
{
    std::string dir = "data/" + prefix + "_values/" + motion_state + "/";
    std::string filename = dir + prefix + "_for_" + sub_motion_state + ".txt";

    if (!boost::filesystem::exists(dir))
    {
        boost::filesystem::create_directories(dir);
    }

    std::ofstream outFile(filename, std::ios_base::app);
    if (outFile.is_open())
    {
        outFile << "Matrix: " << std::endl
                << matrix << std::endl;
        outFile.close();
    }
    else
    {
        std::cerr << "Unable to open the file for writing values." << std::endl;
    }
}

bool UtilityFunctions::checkPathExists(const std::string &path, 
                     const std::string &errorMessage)
{
    struct stat buffer;
    bool exists = (stat(path.c_str(), &buffer) == 0);
    if (!exists)
    {
        std::cout << errorMessage << " Path: " << path << " does not exist. Please change the path.\n";
    }
    return exists;
}

bool UtilityFunctions::SaveData()
{
    bool success = true;

    if (!SavePoseAsKITTI(ref_data_ofs_, current_ref_data_))
    {
        success = false;
        std::cerr << "Error saving ground truth pose." << std::endl;
    }

    if (!SavePosi(ref_data_posi_ofs_, current_ref_data_))
    {
        success = false;
        std::cerr << "Error saving ground truth position." << std::endl;
    }

    if (!SavePoseAsKITTI(eskf_data_ofs, current_pose))
    {
        success = false;
        std::cerr << "Error saving Kalman filter pose." << std::endl;
    }

    if (!SaveSpoofPose(spoof_pose_ofs, current_spoof_pose))
    {
        success = false;
        std::cerr << "Error saving spoofed pose." << std::endl;
    }

    return success;
}

bool UtilityFunctions::SavePoseAsKITTI(std::ofstream &save_points, POSEData &pose) {
    if (!save_points) {
        return false;
    }

    Eigen::Matrix3d rotation_matrix = pose.state.q.toRotationMatrix();

    save_points.precision(12);

    for (int i = 0; i < 3; ++i) {
        save_points << rotation_matrix(i, 0) << " "
                    << rotation_matrix(i, 1) << " "
                    << rotation_matrix(i, 2);
        if (i < 2) {
            save_points << " " << pose.state.p(i) << " ";
        } else {
            save_points << " " << pose.state.p(i) << std::endl;
        }
    }

    return true;
}

bool UtilityFunctions::SavePose(std::ofstream &save_points, 
              POSEData &pose)
{
    if (!save_points)
    {
        return false;
    }
    Eigen::Vector3d euler_angles = utilityFunctions.QuaternionToEulerAngles(pose.state.q);

    save_points.precision(12);

    std::vector<double> data_to_save = {
        pose.time,
        pose.state.p(0), pose.state.p(1), pose.state.p(2),
        pose.state.v(0), pose.state.v(1), pose.state.v(2),
        euler_angles(0), euler_angles(1), euler_angles(2),
        pose.state.bg(0), pose.state.bg(1), pose.state.bg(2),
        pose.state.ba(0), pose.state.ba(1), pose.state.ba(2)};

    for (size_t i = 0; i < data_to_save.size(); ++i)
    {
        save_points << data_to_save[i];
        if (i < data_to_save.size() - 1)
        {
            save_points << ",";
        }
    }
    save_points << std::endl;

    return true;
}

bool UtilityFunctions::SavePosi(std::ofstream &save_points, 
              POSEData &pose)
{
    if (!save_points)
    {
        return false;
    }

    save_points.precision(12);

    std::vector<double> data_to_save = {
        pose.time,
        pose.state.p(0), pose.state.p(1)};

    for (size_t i = 0; i < data_to_save.size(); ++i)
    {
        save_points << data_to_save[i];
        if (i < data_to_save.size() - 1)
        {
            save_points << ",";
        }
    }
    save_points << std::endl;

    return true;
}

void UtilityFunctions::CalculateFrequency()
{
    std::cout << "---------------Initial frequency------------------------" << std::endl;
    CalculateSensorFrequency(gnss_time_stamps, "GNSS");
    CalculateSensorFrequency(imu_time_stamps, "IMU");
    CalculateSensorFrequency(ref_time_stamps, "ref_data_buff_");
    CalculateSensorFrequency(odo_time_stamps, "Odometer");

    if (ref_time_stamps.size() > 1)
    {
        double total_time = ref_time_stamps.back() - ref_time_stamps.front();
        double ref_frequency = static_cast<int>((ref_time_stamps.size() - 1) / total_time);

        std::vector<std::string> ref_names = {"ref_pos", "ref_vel", "ref_att_quat"};
        for (const auto &name : ref_names)
        {
            std::cout << std::left << std::setw(18) << name;
            std::cout << " Frequency: " << std::right << std::setw(11) << std::fixed << ref_frequency << " Hz" << std::endl;
        }
    }
    std::cout << "-" << std::endl;
}

void UtilityFunctions::CalculateSensorFrequency(const std::vector<double> &time_stamps, 
                              const std::string &data_name)
{
    int name_field_width = 18;
    int freq_field_width = 11;

    std::cout << std::left << std::setw(name_field_width) << data_name;
    if (time_stamps.size() > 1)
    {
        double total_time = time_stamps.back() - time_stamps.front();
        double frequency = static_cast<int>((time_stamps.size() - 1) / total_time);
        std::cout << std::right << "Frequency: " << std::setw(freq_field_width) << std::fixed << std::setprecision(2) << frequency << " Hz" << std::endl;
    }
    else
    {
        std::cout << " Not enough data to calculate frequency." << std::endl;
    }
}


bool UtilityFunctions::SaveSpoofPose(std::ofstream &save_points, 
                   SpoofData &pose)
{
    if (!save_points)
    {
        return false;
    }

    save_points.precision(12);

    std::vector<double> data_to_save = {
        pose.time,
        pose.p(0), pose.p(1)};

    for (size_t i = 0; i < data_to_save.size(); ++i)
    {
        save_points << data_to_save[i];
        if (i < data_to_save.size() - 1)
        {
            save_points << ",";
        }
    }
    save_points << std::endl;

    return true;
}
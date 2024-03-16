//load_data.cpp
#include "load_data.h"

// sensor data buff
std::deque<IMUData> imu_data_buff_;
std::deque<GNSSData> gnss_data_buff_;
std::deque<ODOData> odo_data_buff_;
std::deque<POSEData> ref_data_buff_;
// sensor time stamp buff
std::vector<double> imu_time_stamps, gnss_time_stamps, ref_time_stamps, odo_time_stamps;

bool LoadData(const std::vector<std::string> &paths) {
    std::vector<std::ifstream> rawData;
    LoadRawDataBuffers(paths, rawData);

    std::string timeString((std::istreambuf_iterator<char>(rawData[0])),
                                 std::istreambuf_iterator<char>());

    std::istringstream timeDataParser(timeString);
    std::string timeLine;
    std::vector<double> timeStamps;
    while (std::getline(timeDataParser, timeLine)) {
        timeStamps.push_back(std::stod(timeLine));
    }

    for (double time : timeStamps) {
        LoadIMUData(rawData, time);
        LoadOdoData(rawData, time);
        LoadRefData(rawData, time);
    }

    LoadGNSSData(rawData);

    return true;
}

void LoadRawDataBuffers(const std::vector<std::string> &paths, 
                     std::vector<std::ifstream> &fileStreams)
{
    imu_data_buff_.clear();
    imu_time_stamps.clear();
    ref_data_buff_.clear();
    ref_time_stamps.clear();
    odo_data_buff_.clear();
    odo_time_stamps.clear();
    gnss_data_buff_.clear();
    gnss_time_stamps.clear();

    fileStreams.clear();

    for (const auto &path : paths)
    {
        std::ifstream fileStream(path);
        if (!fileStream)
        {
            std::cerr << "Failed to open file: " << path << std::endl;
            continue;
        }

        fileStream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        fileStreams.push_back(std::move(fileStream));
    }
}

void LoadIMUData(std::vector<std::ifstream> &rawData, double &time) {
    std::vector<double> acc, gyro;

    LoadAccGyroData(rawData[1], acc);
    LoadAccGyroData(rawData[2], gyro);

    IMUData imu;
    imu.time = time;
    imu.acc = Eigen::Vector3d(acc[0], acc[1], acc[2]);
    imu.gyro = Eigen::Vector3d(gyro[0], gyro[1], gyro[2]) * DEG_TO_RAD;
    imu_data_buff_.push_back(imu);
    imu_time_stamps.push_back(time);
}

void LoadOdoData(std::vector<std::ifstream> &rawData, 
                 double &time)
{
    std::string strs;
    double odo;

    std::getline(rawData[3], strs);
    odo = std::stod(strs);

    ODOData OdoData;
    OdoData.time = time;
    OdoData.odo = odo;
    odo_data_buff_.push_back(OdoData);
    odo_time_stamps.push_back(time);
}

void LoadAccGyroData(std::ifstream &stream, 
                     std::vector<double> &AccGyroData)
{
    std::string line, temp;
    std::getline(stream, line);
    std::istringstream lineStream(line);
    while (std::getline(lineStream, temp, ','))
    {
        AccGyroData.push_back(std::stod(temp));
    }
}

void LoadRefData(std::vector<std::ifstream> &rawData, 
                 double &time)
{
    std::vector<double> ref_pos, ref_vel, ref_att_quat;

    LoadCSVRefData(rawData[4], ref_pos);
    LoadCSVRefData(rawData[5], ref_vel);
    LoadCSVRefData(rawData[6], ref_att_quat);

    RefDataToPose(ref_pos, ref_vel, ref_att_quat, time);
}

void LoadCSVRefData(std::ifstream &stream, 
                    std::vector<double> &RefData)
{
    std::string line, temp;
    std::getline(stream, line);
    std::istringstream lineStream(line);
    while (std::getline(lineStream, temp, ','))
    {
        RefData.push_back(std::stod(temp));
    }
}

void RefDataToPose(const std::vector<double> &ref_pos, 
                            const std::vector<double> &ref_vel, 
                            const std::vector<double> &ref_att_quat, 
                            double time)
{
    Eigen::Quaterniond q = Eigen::AngleAxisd(90 * DEG_TO_RAD, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(180 * DEG_TO_RAD, Eigen::Vector3d::UnitX());
    q = q.inverse();

    POSEData ref_pose;
    ref_pose.time = time;
    double geo_x, geo_y, geo_z;
    local_geo_transformer.Forward(ref_pos[0], ref_pos[1], ref_pos[2], geo_x, geo_y, geo_z);
    ref_pose.state.p = Eigen::Vector3d(geo_x, geo_y, geo_z);
    ref_pose.state.v = q * Eigen::Vector3d(ref_vel[0], ref_vel[1], ref_vel[2]);
    ref_pose.state.q = q * Eigen::Quaterniond(ref_att_quat[0], ref_att_quat[1], ref_att_quat[2], ref_att_quat[3]);
    ref_pose.state.q.normalize();
    ref_pose.state.bg = Eigen::Vector3d(0, 0, 0);
    ref_pose.state.ba = Eigen::Vector3d(0, 0, 0);

    ref_data_buff_.push_back(ref_pose);
    ref_time_stamps.push_back(time);
}

void LoadGNSSData(std::vector<std::ifstream> &rawData)
{
    std::string timeLine, dataLine;
    while (std::getline(rawData[7], timeLine))
    {
        double time = std::stod(timeLine);

        std::getline(rawData[8], dataLine);
        std::vector<double> gpsData;
        ParseGNSSData(dataLine, gpsData);

        GNSSData gnss;
        gnss.time = time;
        gnss.lla = Eigen::Vector3d(gpsData[0], gpsData[1], gpsData[2]);

        Eigen::Quaterniond q = Eigen::AngleAxisd(90 * DEG_TO_RAD, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(180 * DEG_TO_RAD, Eigen::Vector3d::UnitX());
        q = q.inverse();
        gnss.v = q * Eigen::Vector3d(gpsData[3], gpsData[4], gpsData[5]);

        gnss_data_buff_.push_back(gnss);
        gnss_time_stamps.push_back(time);
    }
}

void ParseGNSSData(const std::string &line, 
                   std::vector<double> &gpsData)
{
    std::istringstream lineStream(line);
    std::string temp;
    while (std::getline(lineStream, temp, ','))
    {
        gpsData.push_back(std::stod(temp));
    }
}

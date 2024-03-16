//load_data.h
#ifndef LOAD_DATA_H
#define LOAD_DATA_H

#include "../../app/utilityFunctions.h"

// sensor data buffer
extern std::deque<IMUData> imu_data_buff_;
extern std::deque<GNSSData> gnss_data_buff_;
extern std::deque<ODOData> odo_data_buff_;
extern std::deque<POSEData> ref_data_buff_;
// sensor time stamp buffer
extern std::vector<double> imu_time_stamps, gnss_time_stamps, ref_time_stamps, odo_time_stamps;
// Degress to Radians
constexpr double DEG_TO_RAD = M_PI / 180.0;

bool LoadData(const std::vector<std::string> &path);
void LoadRawDataBuffers(const std::vector<std::string> &paths,
                     std::vector<std::ifstream> &fileStreams);
bool checkPathExists(const std::string &path, const std::string &path_name);
void LoadIMUData(std::vector<std::ifstream> &rawData,
                 double &time);
void LoadAccGyroData(std::ifstream &stream, 
                     std::vector<double> &AccGyroData);
void LoadCSVRefData(std::ifstream &stream, 
                    std::vector<double> &RefData);
void LoadRefVelData(std::ifstream &refVelStream,
                    std::vector<double> &ref_vel);
void LoadRefAttQuatData(std::ifstream &refAttQuatStream,
                        std::vector<double> &ref_att_quat);
void RefDataToPose(const std::vector<double> &ref_pos,
                            const std::vector<double> &ref_vel,
                            const std::vector<double> &ref_att_quat,
                            double time);
void LoadRefData(std::vector<std::ifstream> &rawData,
                 double &time);
void LoadRefPosData(std::ifstream &refPosStream,
                    std::vector<double> &ref_pos);
void LoadOdoData(std::vector<std::ifstream> &rawData,
                 double &time);
void LoadIMURefODOData(std::vector<std::ifstream> &rawData);
void LoadGNSSData(std::vector<std::ifstream> &rawData);
void ParseGNSSData(const std::string &line, 
                   std::vector<double> &gpsData);

#endif // LOAD_DATA_H
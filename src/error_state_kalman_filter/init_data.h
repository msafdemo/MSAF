//init_data.h
#ifndef INIT_DATA_H
#define INIT_DATA_H

#include "../app/main.h"
#include "../app/utilityFunctions.h"

extern POSEData current_pose;
extern ErrorState current_error_state;
extern GeographicLib::LocalCartesian local_geo_transformer;
// sensor noise
extern std::vector<double> init_dx;
extern std::vector<double> prior;

bool InitSensor();
bool InitPose();
bool InitializePositionAndVelocity();
bool InitializeAttitudeAndBias();
bool InitializeErrorStateAndCov();


#endif // INIT_DATA_H
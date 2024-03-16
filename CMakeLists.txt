# Project setup
project(MSAF)
cmake_minimum_required(VERSION 3.10)

# Include GeographicLib subdirectory and its headers
add_subdirectory(${PROJECT_SOURCE_DIR}/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/GeographicLib/include/)

# Include Eigen3 headers
include_directories(/usr/include/eigen3)

# Setup for YAML-CPP package
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
include_directories(${YAML_CPP_INCLUDEDIR})

# Boost library requirements
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Google glog library for logging
find_package(glog REQUIRED)  

# Executable definition and source files
add_executable(MSAF 
    src/app/main.cpp
    src/motion_data_generator/src/load_data.cpp
    src/motion_data_generator/src/valid_data.cpp
    src/error_state_kalman_filter/init_data.cpp
    src/error_state_kalman_filter/error_state_kalman_filter.cpp 
    src/state_dependency_analyzer/state_dependency_analyzer.cpp
    src/injector/injector.cpp 
    src/app/utilityFunctions.cpp)

# Link libraries to the executable
target_link_libraries(MSAF 
    libGeographiccc 
    ${YAML_CPP_LIBRARIES} 
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    glog::glog) 

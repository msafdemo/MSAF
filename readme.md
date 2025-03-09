# USENIX Security 2025

This repository contains the artifacts associated with the USENIX Security 2025 paper titled **"The Ghost Navigator: Revisiting the Hidden Vulnerabilities of Localization in Autonomous Driving."**

The `Motion Sensitive Analysis Framework (MSAF)` is designed to assess the motion state vulnerabilities in LiDAR-based localization systems.

## Architecture Description

The project consists of four components, each responsible for different aspects of the simulation and analysis:

### `Motion Data Generator`
The `Motion Data Generator` uses simulated data to replicate various vehicle motions and sensor configurations.

- **Input:** Sensor update frequencies, motion configuration files, sensor noise levels.
- **Output:** Raw sensor data in CSV format.

### `Sensor Fusion Engine`
This synthetic dataset is synchronized and processed by the `Sensor Fusion Engine`, which performs essential filtering tasks, such as IMU prediction, and GPS and LiDAR measurement correction.

- **Input:** Raw sensor data.
- **Output:** Results from the Error-State Kalman Filter.

### `State Dependency Analyzer`
Concurrently, the `State Dependency Analyzer` examines the observability of core matrices and variation in Kalman gain for GPS positioning within the sensor fusion process. This component is crucial in identifying and assessing the motion state vulnerabilities within the sensor fusion process.
- **Input:** Measurement time, G, F, and Y matrix of the Error State Kalman Filter.
- **Output:** Observability scores.

### `injector`
The `Injector` simulates GPS spoofing, introducing deviations to assess system resilience against such attacks.

- **Input:** GPS coordinates to be spoofed.
- **Output:** Adjusted GPS coordinates after spoofing.

The observability analysis result, along with the detailed outcomes of the GPS spoofing attacks, are documented in the log files. These files can be found in the `log` directory of the project.

## Installation Guide

### Setting up a Conda Environment
Before initiating the installation, make sure you have Anaconda or Miniconda installed on your system, then create a new conda environment tailored for the `MSAF` project:

1. Open your terminal and create a new conda environment by running:

   ```bash
   conda create --name msaf-env python=3.8
   ```

2. Activate the newly created environment:

   ```bash
   conda activate msaf-env
   ```

### Install Python Dependencies

With the conda environment activated, proceed with the setup:
1. Clone the repository to your local machine.

2. Navigate to the project directory:

   ```bash
   cd path/to/MSAF
   ```

3. Install the required Python libraries by running:

   ```bash
   pip install -r requirements.txt
   ```

### Install gnss-ins-sim
`MSAF` relies on `gnss-ins-sim` to generate critical simulation data for evaluating LiDAR localization. To install the `gnss-ins-sim` library, follow these steps:

1. Navigate to the `gnss-ins-sim` directory within the project:

   ```bash
   cd src/motion_data_generator/gnss-ins-sim
   ```

2. Run the setup script:

   ```bash
   python3 setup.py install
   ```

## Configuration Guide

Modify the `config/config.yaml` file to customize the simulation parameters to your requirements. This includes adjusting sensor noise levels, update frequencies, and selecting fusion strategies to simulate various operational environments and assess their impact on GPS spoofing attack.

Ensure the `path_root`  in `config/config.yaml` matches your project's workspace for accurate resource and data file management.

## Running the Project

To run the project, execute the following command in the project's root directory:

```bash
python3 run.py
```

Note: The setup and testing of `MSAF have been conducted on Ubuntu 18.04.

## Framework Extension

The adaptability of `MSAF` allows for the evaluation of fusion framework robustness against GPS spoofing attacks by integrating new sensors. To implement novel fusion strategies within the system:

1. Design and implement your fusion logic into `error_state_kalman_filter.cpp` and `state_dependency_analyzer.cpp`.
2. Navigate to the build directory:
   ```bash
   cd build
   ```
3. Utilize CMake for setup:
   ```bash
   cmake ..
   ```
4. Compile the updated framework:
   ```bash
   make
   ```
After recompiling, relaunch the framework to analyze the impact of your modifications:

```bash
python3 run.py
```

This step ensures the inclusion of your customized fusion strategies, broadening the analytical capabilities of the framework.

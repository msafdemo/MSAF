import os
import yaml
import math
import numpy as np
from gnss_ins_sim.sim import imu_model  
from gnss_ins_sim.sim import ins_sim 
import glob
import shutil

DEG_TO_RAD = math.pi/180

sensor_config = {
    'imu': {
        'no_error': {
            # 1. gyro:
                # a. random noise:
                    # gyro angle random walk, deg/rt-hr
                    'gyro_arw': np.array([0.00, 0.00, 0.00]) * DEG_TO_RAD/60,
                    # gyro bias instability, deg/hr
                    'gyro_b_stability': np.array([0.00, 0.0, 0.0]) * DEG_TO_RAD/3600.0,
                    # gyro bias isntability correlation time, sec
                    #'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
                # b. deterministic error:
                    'gyro_b': np.array([0.00, 0.00, 0.00]),
                    'gyro_k': np.array([1.00, 1.00, 1.00]),
                    'gyro_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
            # 2. accel:
                # a. random noise:
                    # accel velocity random walk, m/s/rt-hr
                    'accel_vrw': np.array([0.00, 0.00, 0.00]) / 60,
                    # accel bias instability, m/s2
                    'accel_b_stability': np.array([0.00, 0.00, 0.00]),
                    # accel bias isntability correlation time, sec
                    #'accel_b_corr': np.array([100.0, 100.0, 100.0]),
                # b. deterministic error:
                    'accel_b': np.array([0.00, 0.00, 0.00]),
                    'accel_k': np.array([1.00, 1.00, 1.00]),
                    'accel_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
        },
        'high_accuracy': {
            # 1. gyro:
                # a. random noise:
                    # gyro angle random walk, deg/rt-hr
                    'gyro_arw': np.array([0.00, 0.00, 0.00]) * DEG_TO_RAD/60,
                    # gyro bias instability, deg/hr
                    'gyro_b_stability': np.array([0.00, 0.0, 0.0]) * DEG_TO_RAD/3600.0,
                    # gyro bias isntability correlation time, sec
                    #'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
                # b. deterministic error:
                    'gyro_b': np.array([0.00, 0.00, 0.00]),
                    'gyro_k': np.array([1.00, 1.00, 1.00]),
                    'gyro_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
            # 2. accel:
                # a. random noise:
                    # accel velocity random walk, m/s/rt-hr
                    'accel_vrw': np.array([0.00, 0.00, 0.00]) / 60,
                    # accel bias instability, m/s2
                    'accel_b_stability': np.array([0.00, 0.00, 0.00]),
                    # accel bias isntability correlation time, sec
                    #'accel_b_corr': np.array([100.0, 100.0, 100.0]),
                # b. deterministic error:
                    'accel_b': np.array([0.00, 0.00, 0.00]),
                    'accel_k': np.array([1.00, 1.00, 1.00]),
                    'accel_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
        },
        'mid_accuracy': {
            # 1. gyro:
                # a. random noise:
                    # gyro angle random walk, deg/rt-hr
                    'gyro_arw': np.array([2.0e-3, 2.0e-3, 2.0e-3]) * DEG_TO_RAD/60,
                    # gyro bias instability, deg/hr
                    'gyro_b_stability': np.array([0.1, 0.1, 0.1]) * DEG_TO_RAD/3600.0,
                    # gyro bias isntability correlation time, sec
                    'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
                # b. deterministic error:
                    'gyro_b': np.array([0.0, 0.0, 0.0]) * DEG_TO_RAD,
                    'gyro_k': np.array([1.00, 1.00, 1.00]),
                    'gyro_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
            # 2. accel:
                # a. random noise:
                    # accel velocity random walk, m/s/rt-hr
                    'accel_vrw': np.array([2.5e-5, 2.5e-5, 2.5e-5]) / 60,
                    # accel bias instability, m/s2
                    'accel_b_stability': np.array([3.6e-6, 3.6e-6, 3.6e-6]),
                    # accel bias isntability correlation time, sec
                    'accel_b_corr': np.array([100.0, 100.0, 100.0]),
                # b. deterministic error:
                    'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
                    'accel_k': np.array([1.00, 1.00, 1.00]),
                    'accel_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
        },
        'low_accuracy': {
            # 1. gyro:
                # a. random noise:
                    # gyro angle random walk, deg/rt-hr
                    'gyro_arw': np.array([2.0e-3, 2.0e-3, 2.0e-3]) * DEG_TO_RAD/60,
                    # gyro bias instability, deg/hr
                    'gyro_b_stability': np.array([0.1, 0.1, 0.1]) * DEG_TO_RAD/3600.0,
                    # gyro bias isntability correlation time, sec
                    'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
                # b. deterministic error:
                    'gyro_b': np.array([0.0, 0.0, 0.0]) * DEG_TO_RAD,
                    'gyro_k': np.array([1.00, 1.00, 1.00]),
                    'gyro_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
            # 2. accel:
                # a. random noise:
                    # accel velocity random walk, m/s/rt-hr
                    'accel_vrw': np.array([2.5e-5, 2.5e-5, 2.5e-5]) / 60,
                    # accel bias instability, m/s2
                    'accel_b_stability': np.array([3.6e-6, 3.6e-6, 3.6e-6]),
                    # accel bias isntability correlation time, sec
                    'accel_b_corr': np.array([100.0, 100.0, 100.0]),
                # b. deterministic error:
                    'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
                    'accel_k': np.array([1.00, 1.00, 1.00]),
                    'accel_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
        }
    },
    'gps': {
        'no_error': {
            'stdp': np.array([0.0, 0.0, 0.0]),
            'stdv': np.array([0.0, 0.0, 0.0])
        },
        'high_accuracy': {
            'stdp': np.array([0.01, 0.01, 0.01]),
            'stdv': np.array([0.01, 0.01, 0.01])
        },
        'mid_accuracy': {
            'stdp': np.array([0.1, 0.1, 0.1]),
            'stdv': np.array([0.02, 0.02, 0.02])
        },
        'low_accuracy': {
            'stdp': np.array([0.5, 0.5, 0.5]),
            'stdv': np.array([0.05, 0.05, 0.05])
        }
    },
    'odo': {
        'no_error': {
            'scale': 1.00,
            'stdv': 0.0
        },
        'high_accuracy': {
            'scale': 1.00,
            'stdv': 0.05
        },
        'mid_accuracy': {
            'scale': 1.00,
            'stdv': 0.1
        },
        'low_accuracy': {
            'scale': 1.00,
            'stdv': 0.5
        }
    }
}

def read_config(file_path):
    with open(file_path, 'r') as yamlfile:
        return yaml.load(yamlfile, Loader=yaml.FullLoader)

def clear_folder(folder_path):
    for filename in os.listdir(folder_path):
        file_path = os.path.join(folder_path, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print(f"Failed to delete {file_path}. Reason: {e}")

def get_csv_filenames(folder_path):
    all_files = os.listdir(folder_path)
    csv_files = [os.path.splitext(file)[0] for file in all_files if file.endswith('.csv')]
    return csv_files

def check_and_create_path(path):
    if not os.path.exists(path):
        print(f"Path {path} does not exist. Creating it.")
        os.makedirs(path)

def run_simulation(sensor_update_rates, motion_state, imu, motion_def_full_path, sim_data_full_path):
    search_pattern = os.path.join(motion_def_full_path, "*.csv")
    motion_files = glob.glob(search_pattern)
    clear_folder(sim_data_full_path)
    if not motion_files:
        print(f"No .csv files found in {motion_def_full_path}. Skipping this motion state.")
        return
    
    for motion_file in motion_files:
        motion_state_name = os.path.splitext(os.path.basename(motion_file))[0]
        result_path = os.path.join(sim_data_full_path, motion_state_name)

        sim = ins_sim.Sim(sensor_update_rates,
                          motion_file,
                          ref_frame=0,
                          imu=imu,
                          mode=None,
                          env=None,
                          algorithm=None)
        sim.run(1)
        sim.results(result_path)

def path_gen(sensor_update_rates, 
             imu_error_level, gps_error_level, odo_error_level, 
             config, motion_def_full_path, sim_data_full_path, motion_states):
    imu_err = config['imu'][imu_error_level].copy()
    print('imu_err: ', imu_err)
    gps_err = config['gps'][gps_error_level]
    print('gps_err: ', gps_err)
    odo_err = config['odo'][odo_error_level]
    print('odo_err: ', odo_err)
    
    imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=True, gps_opt=gps_err, odo=True, odo_opt=odo_err)
    
    for state in motion_states:
        run_simulation(sensor_update_rates, state, imu, motion_def_full_path, sim_data_full_path)

def main():
    config = read_config('config/config.yaml')
    workspace = config['path_root']['workspace']
    sensor_update_rates = [config['sensor_update_rates']['imu_hz'], 
                           config['sensor_update_rates']['gps_hz'], 
                           config['sensor_update_rates']['odo_hz']]
    
    sensor_noise_levels_keys = ['imu_noise', 'gps_noise', 'odom_vel_noise']
    for key in sensor_noise_levels_keys:
        if key not in config['sensor_noise_levels']:
            config['sensor_noise_levels'][key] = 'no_error'

    motion_state = config.get('motion_state', 'straight_vel')
    
    motion_def_folder = f"src/motion_data_generator/motion_files/{motion_state}"
    motion_def_full_path = os.path.join(workspace, motion_def_folder)
    motion_states = get_csv_filenames(motion_def_full_path)
    print(f"motion_states: {motion_states}")

    if motion_states:
        motion_def_folder = f"src/motion_data_generator/motion_files/{motion_state}" 
        sim_data_folder = f"data/sim_data/{motion_state}"
        motion_def_full_path = os.path.join(workspace, motion_def_folder)
        sim_data_full_path = os.path.join(workspace, sim_data_folder)
        check_and_create_path(motion_def_full_path)
        check_and_create_path(sim_data_full_path)

        if not os.listdir(motion_def_full_path):
            print(f"Error: The folder {motion_def_full_path} is empty.")
            return
        
        print(f"motion states: {motion_states}")  

        path_gen(sensor_update_rates, 
                 config['sensor_noise_levels']['imu_noise'],
                 config['sensor_noise_levels']['gps_noise'], 
                 config['sensor_noise_levels']['odom_vel_noise'], 
                 sensor_config, motion_def_full_path, sim_data_full_path, motion_states)
    else:
        print(f"Motion set {motion_state} not found in config.")

if __name__ == '__main__':
    main()

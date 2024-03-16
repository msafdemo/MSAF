import subprocess
import yaml
import os
import shutil
import re

def get_csv_filenames(folder_path):
    all_files = os.listdir(folder_path)
    csv_files = [os.path.splitext(file)[0] for file in all_files if file.endswith('.csv')]
    return csv_files
def alphanumeric_key(s):
    return [int(text) if text.isdigit() else text for text in re.split('([0-9]+)', s)]

def run_cpp_program(config_data, state, common_file_name=None):
    temp_file_path = os.path.join(config_data['path_root']['workspace'], config_data['paths_for_eskf']['temp_file_for_eskf_batch'])
    with open(temp_file_path, 'w') as file:
        yaml.safe_dump({'motion_state': state}, file)

    obs_result_dir = os.path.dirname(config_data['paths_for_eskf']['obs_result_file_path'])
    if common_file_name:
        common_file_path = os.path.join(obs_result_dir, "obs_result.txt")
        command = [config_data['paths_for_eskf']['eskf_build_path'], state, "", common_file_path]
    else:
        command = [config_data['paths_for_eskf']['eskf_build_path'], state, ""]

    print("Debug: command:", command)
    result = subprocess.run(command)
    print("Debug: result:", result)

def read_and_init_config(path):
    if not os.path.exists(path):
        print("The YAML file does not exist.")
        return None

    with open(path, 'r') as file:
        config_data = yaml.safe_load(file)

    workspace = config_data['path_root']['workspace']
    for key, value in config_data['paths_for_eskf'].items():
        config_data['paths_for_eskf'][key] = os.path.join(workspace, value)

    obs_result_dir = os.path.dirname(config_data['paths_for_eskf']['obs_result_file_path'])
    print("Debug: obs_result_dir:", obs_result_dir)

    if os.path.exists(obs_result_dir):
        for filename in os.listdir(obs_result_dir):
            print("Debug: filename:", filename)
            file_path = os.path.join(obs_result_dir, filename)
            print("Debug: file_path:", file_path)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print(f"Failed to delete {file_path}. Reason: {e}")

    if not os.path.exists(obs_result_dir):
        os.makedirs(obs_result_dir)

    return config_data

def main():
    print("------------------------module.2.Sensor Fusion Engine start---------------------")

    config_path = 'config/config.yaml'
    config_data = read_and_init_config(config_path)
    if config_data is None:
        print("Exiting due to the missing config file.")
        return

    g_values_path = os.path.join(config_data['path_root']['workspace'], "data/G_values")
    k_values_path = os.path.join(config_data['path_root']['workspace'], "data/K_values")
    chi_square_path = os.path.join(config_data['path_root']['workspace'], "data/chi_square")
    
    if os.path.exists(g_values_path):
        shutil.rmtree(g_values_path)
    
    if os.path.exists(k_values_path):
        shutil.rmtree(k_values_path)

    if os.path.exists(chi_square_path):
        shutil.rmtree(chi_square_path)

    motion_state = config_data.get('motion_state', 'straight_vel')
    
    # Get motion_states based on motion_state
    motion_def_folder = f"src/motion_data_generator/motion_files/{motion_state}"
    motion_def_full_path = os.path.join(config_data['path_root']['workspace'], motion_def_folder)
    motion_states = get_csv_filenames(motion_def_full_path)
    motion_states = sorted(motion_states, key=alphanumeric_key)
    
    print("Debug: motion_states:", motion_states)

    common_file_name = "obs_result.txt"

    for state in motion_states:
        print("Debug: state:", state)
        run_cpp_program(config_data, state, common_file_name)

    print("------------------------module.2.Sensor Fusion Engine Done.---------------------")

if __name__ == '__main__':
    main()

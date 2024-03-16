from collections import defaultdict
import os
import yaml
import re

state_to_label = {
    0: 'Px', 1: 'Py', 2: 'Pz', 3: 'Vx', 4: 'Vy', 5: 'Vz',
    6: 'Rx', 7: 'Ry', 8: 'Rz', 9: 'GBx', 10: 'GBy', 11: 'GBz',
    12: 'ABx', 13: 'ABy', 14: 'ABz'
}
def custom_sort_key(item):
    number = float(re.search(r'(\d+\.\d+|\d+)', item).group(1))
    prefix_init_dxity = 0 if 'observability_' in item else 1
    return (prefix_init_dxity, number)  


def get_csv_filenames(path):
    return [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f)) and f.endswith('.csv')]

def load_config(config_path):
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config

def is_new_category_line(line):
    return 'motion_state' in line and 'State' not in line and 'SingularValue' not in line and 'Observability' not in line

def read_and_parse_txt_file(obs_result_in_txt_path, use_state_label):
    parsed_data_by_category = defaultdict(list)
    current_category = None

    with open(obs_result_in_txt_path, 'r') as f:
        content = f.readlines()

    for line in content:
        line = line.strip()
        if not line:
            continue

        if is_new_category_line(line):
            current_category = line.split(':')[1].strip().lower()
            continue

        if current_category and ',' in line:
            parts = line.split(',')
            state = int(parts[0].split()[-1])
            observability = float(parts[-1].split(':')[-1].strip())
            
            if use_state_label:
                label = state_to_label.get(state, f"unknown_{state}")
                parsed_data_by_category[current_category].append({'state': label, 'observability': observability})
            else:
                parsed_data_by_category[current_category].append({'state': state, 'observability': observability})

    return parsed_data_by_category

def Handle_parsed_data(parsed_data):
    Handled_data = {}
    for category, data in parsed_data.items():
        Handled_data[f'observability_{category}'] = [item['observability'] for item in data]
        Handled_data[f'state_{category}'] = [item['state'] for item in data]

    return Handled_data

def write_to_yaml_file(Handled_data, obs_result_in_yaml_path):
    yaml_result_dir = os.path.dirname(obs_result_in_yaml_path)
    if not os.path.exists(yaml_result_dir):
        os.makedirs(yaml_result_dir)

    if os.path.exists(obs_result_in_yaml_path):
        os.remove(obs_result_in_yaml_path)

    sorted_outer_keys = sorted(Handled_data.keys(), key=lambda x: (x.split('_')[-1], custom_sort_key(x)))

    sorted_data = {}
    for key in sorted_outer_keys:
        sorted_data[key] = Handled_data[key]

    output_yaml = yaml.dump(sorted_data, allow_unicode=True, default_flow_style=False, sort_keys=False)

    with open(obs_result_in_yaml_path, 'w') as f:
        f.write(output_yaml)

def main():
    print("------------------------module.3.State Dependency Analysis start---------------")
    
    config = load_config("config/config.yaml")
    workspace = config['path_root']['workspace']
    use_state_label = config['use_state_label']
    motion_state = config.get('motion_state', 'straight_vel')
    
    obs_result_in_txt_path = os.path.join(workspace, config['paths_for_eskf']['obs_result_file_path'] + 'obs_result.txt')
    parsed_data = read_and_parse_txt_file(obs_result_in_txt_path, use_state_label)
    
    Handled_data = Handle_parsed_data(parsed_data)
    obs_result_in_yaml_path = os.path.join(workspace, config['paths_for_parse_obs_to_yaml']['yaml_result_path'] + f'{motion_state}.yaml')
    write_to_yaml_file(Handled_data, obs_result_in_yaml_path)

    print("------------------------module.3.State Dependency Analysis Done----------------")

if __name__ == "__main__":
    main()

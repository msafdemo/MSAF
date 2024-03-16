import os
import shutil
import csv
import yaml

existing_suffixes = set()

def format_value(value):
    if value is None:
        return None
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return str(int(value)) if value.is_integer() else str(value)
    return str(value)

def read_straight_vel_value(csv_path):
    with open(csv_path, 'r') as file:
        csvreader = csv.reader(file)
        try:
            headers = next(csvreader)  
            init_values = next(csvreader)  
            command_headers = next(csvreader) 
        except StopIteration:
            print("Unexpected end of file in {}".format(csv_path))
            return None, None, None

        value = None
        command_type = None
        command_duration = None

        try:
            if "ini vy_body (m/s)" in headers:
                index = headers.index("ini vy_body (m/s)")
                if init_values[index].replace('.', '', 1).isdigit():
                    value = float(init_values[index])
                else:
                    print(f"Unexpected value: {init_values[index]}")

            if "command type" in command_headers:
                index = command_headers.index("command type")
                command_values = next(csvreader)
                command_type = "c" + str(command_values[index])

            if "command duration (s)" in command_headers:
                index = command_headers.index("command duration (s)")
                command_duration = int(float(command_values[index]))

        except ValueError as e:
            print(f"Error: {e}")
            return None, None, None

        return value, command_type, command_duration

def read_straight_acc_value(csv_path):
    with open(csv_path, 'r') as file:
        csvreader = csv.reader(file)
        headers = next(csvreader)
        init_values = next(csvreader)
        command_headers = next(csvreader)
        command_values = next(csvreader)
        ini_vy_body, vy_body = 0.0, 0.0
        command_type = None
        if "ini vy_body (m/s)" in headers:
            index = headers.index("ini vy_body (m/s)")
            ini_vy_body = float(init_values[index])
        if "vy_body (m/s)" in command_headers:
            index = command_headers.index("vy_body (m/s)")
            vy_body = float(command_values[index])
        if "command type" in command_headers:
            index = command_headers.index("command type")
            command_type = "c" + str(command_values[index])
        if "command duration (s)" in command_headers:
            command_duration_index = command_headers.index(
                "command duration (s)")
            command_duration = int(
                float(command_values[command_duration_index]))
        return ini_vy_body, vy_body, command_type, command_duration

def read_turning_yaw_rate_value(csv_path):
    with open(csv_path, 'r') as file:
        csvreader = csv.reader(file)
        next(csvreader)
        next(csvreader)
        command_headers = next(csvreader)
        command_values = next(csvreader)
        yaw_value = None
        command_type = None
        command_duration = None
        if "yaw (deg)" in command_headers:
            yaw_index = command_headers.index("yaw (deg)")
            yaw_value = float(command_values[yaw_index])
        if "command type" in command_headers:
            command_type_index = command_headers.index("command type")
            command_type = "c" + str(command_values[command_type_index])
        if "command duration (s)" in command_headers:
            command_duration_index = command_headers.index(
                "command duration (s)")
            command_duration = int(
                float(command_values[command_duration_index]))
        return yaw_value, command_type, command_duration

def rename_file(base_dir, original_filename, new_suffix, command_type, command_duration, motion_state, ini_vy_body=None, vy_body=None):
    new_suffix_str = format_value(new_suffix)
    command_duration_str = format_value(command_duration) if command_duration is not None else 'N/A'
    
    ini_vy_body_str = ""
    if ini_vy_body is not None:
        ini_vy_body_str = f"{format_value(ini_vy_body)}mps"

    vy_body_str = ""
    if vy_body is not None:
        vy_body_str = f"{format_value(vy_body)}mpss"
    
    unit_str = ""
    if motion_state in ["turning_yaw", "turning_yaw_vel"]:
        if command_type == "c1":
            unit_str = "degps"
        elif command_type == "c2":
            unit_str = "deg"

    if motion_state == "straight_vel":
        new_filename = f"{new_suffix_str}mps.csv"
    elif motion_state == "straight_acc":
        new_filename = f"{vy_body_str}.csv"
    elif motion_state == "turning_yaw":
        new_filename = f"{new_suffix_str}{unit_str}.csv"
    elif motion_state == "turning_yaw_vel":
        new_filename = f"{new_suffix_str}{unit_str}_{ini_vy_body_str}.csv"
    else:
        new_filename = f"{ini_vy_body_str}{new_suffix_str}{unit_str}.csv"

    if new_filename in existing_suffixes:
        return

    existing_suffixes.add(new_filename)
    original_path = os.path.join(base_dir, original_filename)
    new_path = os.path.join(base_dir, new_filename)
    shutil.move(original_path, new_path)

def Handle_files(motion_state, dir_path, write_config_data):
    filenames = [f for f in os.listdir(dir_path) if f.endswith('.csv')]
    existing_suffixes.clear()
    values = []

    for filename in filenames:
        csv_path = os.path.join(dir_path, filename)

        if motion_state == "straight_vel":
            value, command_type, command_duration = read_straight_vel_value(csv_path)
            if value is not None:
                rename_file(dir_path, filename, value, command_type, command_duration, motion_state)

        elif motion_state == "straight_acc":
            ini_vy_body, vy_body, command_type, command_duration = read_straight_acc_value(csv_path)
            rename_file(dir_path, filename, None, command_type, command_duration, motion_state, ini_vy_body, vy_body)

        elif motion_state in ["turning_yaw", "turning_yaw_vel"]:
            yaw_value, command_type, command_duration = read_turning_yaw_rate_value(csv_path)
            ini_vy_body = None  
            with open(csv_path, 'r') as file:
                csvreader = csv.reader(file)
                headers = next(csvreader)
                init_values = next(csvreader)
                if "ini vy_body (m/s)" in headers:
                    index = headers.index("ini vy_body (m/s)")
                    ini_vy_body = float(init_values[index])
            rename_file(dir_path, filename, yaw_value, command_type, command_duration, motion_state, ini_vy_body)

        if motion_state == "straight_vel" and value is not None:
            values.append(value)
        elif motion_state == "straight_acc" and ini_vy_body is not None and vy_body is not None:
            values.append(f"{ini_vy_body}_{vy_body}")
        elif motion_state in ["turning_yaw", "turning_yaw_vel"]  and yaw_value is not None:
            values.append(yaw_value)

    values.sort()
    key_to_update = f"suffixes_to_add_for_{motion_state}"
    write_config_data[key_to_update] = values

def main():
    read_yaml_filepath = 'config/config.yaml'
    write_yaml_filepath = 'config/generator_config.yaml'

    with open(read_yaml_filepath, 'r') as yaml_file:
        read_config_data = yaml.safe_load(yaml_file)

    motion_state = read_config_data.get("motion_state", "")

    if not motion_state:
        return

    try:
        with open(write_yaml_filepath, 'r') as yaml_file:
            write_config_data = yaml.safe_load(yaml_file)
    except FileNotFoundError:
        write_config_data = {}

    dir_path = f"src/motion_data_generator/motion_files/{motion_state}/"
    Handle_files(motion_state, dir_path, write_config_data)

    os.makedirs(os.path.dirname(write_yaml_filepath), exist_ok=True)

    with open(write_yaml_filepath, 'w') as yaml_file:
        yaml.dump(write_config_data, yaml_file, default_flow_style=False)


if __name__ == "__main__":
    main()

# /usr/bin/python3.8
import subprocess
import os
import yaml
import logging
import datetime
import glob

def upgrade_libstdcxx():
    """
    Upgrades libstdc++ to ensure support for GLIBCXX_3.4.29.
    """
    print("Checking and upgrading libstdc++ if needed...")

    # Check current GLIBCXX versions
    check_command = "strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX"
    try:
        result = subprocess.run(check_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        output = result.stdout
        if "GLIBCXX_3.4.29" in output:
            print("libstdc++ is already up-to-date.")
            return
        else:
            print("libstdc++ does not support GLIBCXX_3.4.29. Proceeding with upgrade.")
    except subprocess.CalledProcessError as e:
        print("Failed to check libstdc++ version. Please ensure it exists.")
        return

    # Add PPA and upgrade
    commands = [
        "sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test",
        "sudo apt update",
        "sudo apt install -y libstdc++6"
    ]
    for command in commands:
        try:
            print(f"Running command: {command}")
            result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            print(result.stdout)
        except subprocess.CalledProcessError as e:
            print(f"Command failed: {command}")
            return

    # Re-check libstdc++ version
    try:
        result = subprocess.run(check_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        output = result.stdout
        if "GLIBCXX_3.4.29" in output:
            print("Successfully upgraded libstdc++ to support GLIBCXX_3.4.29.")
        else:
            print("Failed to upgrade libstdc++ to support GLIBCXX_3.4.29.")
    except subprocess.CalledProcessError:
        print("Failed to re-check libstdc++ version.")


def get_execution_steps(motion_state):
    steps = [
        # model1:motion_data_generator
        ("Adaptively rename motion files based on parameters", "src/motion_data_generator/src/param_adapter.py"),
        ("Data Generation", "src/motion_data_generator/src/generator.py"),
        ("Plot the Simulated Path from motion_data_generator", "scripts/plot_ref_path.py"),
        
        # model2&3:eskf & obs
        ("Execute ESKF and observability analysis", "scripts/eskf_batch.py"),
        ("Parse Observations", "scripts/parse_obs_to_yaml.py"),
        ("Plot Radar", "scripts/plot_radar.py"),
        ("Plot K Values", "scripts/plot_k_values.py"),
    ]
    
        # model4:spoofing
    if motion_state == 'turning_yaw':
        steps.append(("Plot Spoof_path", "scripts/plot_spoof_path_yaw.py"))
    elif motion_state == 'turning_yaw_vel':
        steps.append(("Plot Spoof_path", "scripts/plot_spoof_path_yaw_vel.py"))
    elif motion_state in ['straight_vel', 'straight_acc']:
        steps.append(("Plot Spoof_path", "scripts/plot_spoof_path_straight.py"))
    else:
        print(f"Warning: Unsupported motion_state '{motion_state}', skipping Plot Spoof_path step.")

    return steps

def clean_old_logs(log_folder):
    log_files = glob.glob(f"{log_folder}/run_*.log")
    if len(log_files) > 10:
        log_files = sorted(log_files, key=os.path.getmtime, reverse=True)
        for log_file in log_files[10:]:
            os.remove(log_file)

def rename_latest_log(log_folder):
    latest_log_path = os.path.join(log_folder, 'latest.log')
    if os.path.exists(latest_log_path):
        timestamp = datetime.datetime.now().strftime('%m%d_%H%M%S')
        new_name = f'run_{timestamp}.log'
        os.rename(latest_log_path, os.path.join(log_folder, new_name))

def initialize_logging():
    log_folder = 'log'
    rename_latest_log(log_folder)

    log_filename = os.path.join(log_folder, 'latest.log')

    clean_old_logs(log_folder)

    logging.basicConfig(filename=log_filename, level=logging.INFO,
                        format='%(asctime)s %(levelname)s: %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')
    
def read_config(file_path):
    try:
        with open(file_path, 'r') as yamlfile:
            return yaml.safe_load(yamlfile)
    except Exception as e:
        logging.error(f"Error reading YAML: {e}")
        return None

def run_command(command):
    try:
        result = subprocess.run(command, shell=True, check=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        output = result.stdout
        print(output)
        logging.info(f"Execution successful: {command}\n{output}")
        return True
    except subprocess.CalledProcessError as e:
        error_output = e.stdout
        print(error_output)
        logging.error(f"Execution failed: {command}\n{error_output}")
        return False


def execute_steps(workspace, steps):
    for step_name, script_path in steps:
        full_path = os.path.join(workspace, script_path)
        logging.info(f"Executing Step: {step_name}")

        if not run_command(f"python {full_path}"):
            logging.error(
                f"Step '{step_name}' failed. Aborting the remaining steps.")
            break

def main():
    initialize_logging()

    upgrade_libstdcxx()

    config_path = "config/config.yaml"
    cfg = read_config(config_path)

    if not cfg:
        logging.error("Failed to read the configuration file.")
        return

    workspace = cfg.get('path_root', {}).get('workspace', "")
    motion_state = cfg.get('motion_state', '')

    if not workspace:
        logging.error("Workspace path is not specified in the config file.")
        return

    print("motion_state: ", motion_state)
    
    steps = get_execution_steps(motion_state)
    execute_steps(workspace, steps)

    print("------------------------All Done.----------------------------------------")

if __name__ == "__main__":
    main()

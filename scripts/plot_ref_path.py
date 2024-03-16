import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import yaml
import numpy as np
import pandas as pd
import re

def natural_keys(text):
    return [int(c) if c.isdigit() else c for c in re.split('(\d+)', text)]

def read_config(file_path):
    try:
        with open(file_path, 'r') as yamlfile:
            return yaml.safe_load(yamlfile)
    except Exception as e:
        print(f"Error reading YAML: {e}")
        return None

def read_ref_pos_trajectory(file_path):
    try:
        df = pd.read_csv(file_path)
        lat_vals = df['ref_pos_lat (deg)'].tolist()
        lon_vals = df['ref_pos_lon (deg)'].tolist()
        alt_vals = df['ref_pos_alt (m)'].tolist()
        return lat_vals, lon_vals, alt_vals
    except Exception as e:
        print(f"Error reading ref_pos file {file_path}: {e}")
        return None, None, None

def convert_to_enu(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    lat = np.array(lat)
    lon = np.array(lon)
    alt = np.array(alt)
    
    R_earth = 6371e3  
    x = (lat - lat_ref) * (np.pi / 180) * R_earth
    y = (lon - lon_ref) * (np.pi / 180) * R_earth * np.cos(lat * np.pi / 180)
    z = alt - alt_ref
    return x.tolist(), y.tolist(), z.tolist()

def setup_plot(motion_types, dimension):
    num_plots = len(motion_types)
    if num_plots == 0:
        return None
    
    if num_plots == 1:
        rows = 1
        cols = 1
    elif num_plots % 2 == 0:
        rows = 2
        cols = num_plots // 2
    else:
        # rows = 3
        # cols = int(np.ceil(num_plots / 3))
        rows = 3
        cols = 3
        
    fig, axs = plt.subplots(rows, cols, figsize=(15, 10), subplot_kw={'projection': dimension.lower() if dimension.lower() == "3d" else None})
    return axs.flatten() if rows > 1 or cols > 1 else [axs]

def adjust_values(values, threshold=1e-3):
    adjusted = [values[0]]
    for i in range(1, len(values)):
        if np.linalg.norm(np.array(values[i]) - np.array(values[i-1])) < threshold:
            adjusted.append(adjusted[-1])
        else:
            adjusted.append(values[i])
    return adjusted

def plot_trajectory(ax, x, y, z, motion_type, motion_state, dimension):
    if dimension == "3D":
        ax.plot(x, y, z)
        ax.set_zlabel('Z')
    else:
        ax.plot(x, y)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(f'{motion_type}', color='blue')

def on_key(event):
    plt.close()

def on_mouse_click(event):
    plt.close()

def load_config_and_prepare_data():
    config_path = 'config/config.yaml'
    cfg = read_config(config_path)
    if cfg:
        dimension = cfg.get('plot_dimension', '3D')
        filter_threshold = float(cfg.get('filter_threshold', 1e-3))
        motion_state = cfg.get('motion_state', 'debug')
        enable_z_adjust = cfg.get('enable_z_adjust', True)
    else:
        dimension = '3D'
        filter_threshold = 1e-3
        motion_state = 'debug'
        enable_z_adjust = True

    sim_data_path = os.path.join("data", "sim_data", f"{motion_state}")
    sub_folders = sorted([name for name in os.listdir(sim_data_path) if os.path.isdir(os.path.join(sim_data_path, name))], key=natural_keys)

    return dimension, filter_threshold, sim_data_path, sub_folders, enable_z_adjust, motion_state, sub_folders

def main():
    print("------------------------module.1.motion_data_generator start------------------")
    dimension, filter_threshold, sim_data_path, sub_folders, enable_z_adjust, motion_state, selected_motion_types = load_config_and_prepare_data()
    
    axs = setup_plot(sub_folders, dimension)
    if axs is None:
        print("No plots to generate.")
        return
    
    ax_index = 0
    for sub_folder in sub_folders:
        file_path = os.path.join(sim_data_path, sub_folder, 'ref_pos.csv')
        lat, lon, alt = read_ref_pos_trajectory(file_path)
        if lat is not None and len(lat) > 0:
            lat_ref, lon_ref, alt_ref = lat[0], lon[0], alt[0]
            x, y, z = convert_to_enu(lat, lon, alt, lat_ref, lon_ref, alt_ref)
            if enable_z_adjust:
                z = adjust_values(z, filter_threshold)
            plot_trajectory(axs[ax_index], x, y, z, sub_folder, motion_state, dimension)
            ax_index += 1

    for i in range(ax_index, len(axs)):
        axs[i].remove()

    save_path = os.path.join(f"img/{motion_state}/", f"{motion_state}" + "_path.png")
    save_directory = "img"
    if not os.path.exists(save_directory):
        os.makedirs(save_directory)

    plt.tight_layout(rect=[0, 0.1, 1, 0.9])
    # plt.savefig(save_path)
    plt.gcf().canvas.mpl_connect('key_press_event', on_key)
    plt.show()
    
    print("------------------------module.1.motion_data_generator Done.------------------")

if __name__ == "__main__":
    main()

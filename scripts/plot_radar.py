import matplotlib.pyplot as plt
import numpy as np
import yaml
import os
import re

def load_yaml_data(filepath):
    with open(filepath, 'r') as yaml_file:
        return yaml.load(yaml_file, Loader=yaml.FullLoader)

def on_key(event):
    plt.close()

def natural_keys(text):
    """ Function to sort strings with numbers, including floating point and negative numbers in them. """
    return [float(c) if c.replace('.', '', 1).isdigit() else c for c in re.split('([0-9.]+)', text)]

def on_mouse_click(event):
    plt.close()

def alphanumeric_sort_key(item):
    """ Function to sort strings with numbers, including floating point and negative numbers in them. """
    def convert(text):
        if text.replace('.', '', 1).replace('-', '', 1).isdigit():
            return float(text)
        else:
            return text.lower()
    
    parts = re.split('(-?[0-9]+\.?[0-9]*)', item)
    
    return [convert(part) for part in parts if part]

def prepare_titles(titles):
    return titles

def get_subplots_layout(num_data):
    """Determine number of rows and columns for subplots based on the number of datasets."""
    if num_data <= 3:
        return 1, num_data
    elif num_data <= 6:
        return 1, num_data
    else:
        return 3, int(np.ceil(num_data / 3.0))


def plot_single_chart(ax, state, observability, title, color):
    angles = np.linspace(0, 2 * np.pi, len(state), endpoint=False).tolist()
    angles.append(angles[0])
    observability = np.append(observability, observability[0])
    ax.plot(angles, observability, marker='o', linestyle='-', color=color, markersize=8, label=title)
    ax.fill(angles, observability, alpha=0.25, facecolor=color)
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(state, fontsize=32)

    ax.tick_params(axis='x', which='major', pad=16)
    ax.set_ylim([0, 1])
    ax.set_rgrids([0.2, 0.4, 0.6, 0.8], angle=0, fontsize=18)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), fontsize=35)


def plot_radar_charts(axs, states, observabilities, titles, colors):
    titles = prepare_titles(titles)
    for ax, state, observability, title, color in zip(axs, states, observabilities, titles, colors):
        plot_single_chart(ax, state, observability, title, color)



def get_csv_filenames(folder_path):
    all_files = os.listdir(folder_path)
    csv_files = [os.path.splitext(file)[0] for file in all_files if file.endswith('.csv')]
    return csv_files

def load_data(config_data):
    motion_state = config_data.get('motion_state', 'straight_vel')
    data_folder = "data/parse_result"
    data_filepath = os.path.join(data_folder, f"{motion_state}.yaml")

    if not os.path.exists(data_filepath):
        print(f"file '{data_filepath}' does not exist")
        return [], [], [], {}
    
    motion_data = load_yaml_data(data_filepath)
    
    state_keys = [key for key in motion_data.keys() if key.startswith('state_')]
    state_keys = sorted(state_keys, key=natural_keys)
    
    # make sure the observability keys are in the same order as the state keys
    observability_keys = [key.replace('state_', 'observability_') for key in state_keys]
    
    states, observabilities, titles = [], [], []
    for state_key, observability_key in zip(state_keys, observability_keys):
        states.append(np.array(motion_data[state_key]))
        observabilities.append(np.array(motion_data[observability_key]))
        titles.append(state_key.replace('state_', '').capitalize())

    sensor_update_rates = config_data.get('sensor_update_rates', {})
    
    return states, observabilities, titles, sensor_update_rates


def prepare_plot_elements(config_data, sensor_update_rates):
    fusion_strategy = config_data.get('fusion_strategy', False)
    if fusion_strategy == "gnss_odom_lidar":
        sub_title = "IMU+ODOM+GPS+LIDAR"
    elif fusion_strategy == "gnss_odom":
        sub_title = "IMU+ODOM+GPS"
    elif fusion_strategy == "gnss_lidar":
        sub_title = "IMU+LIDAR+GPS"
    elif fusion_strategy == "gnss":
        sub_title = "IMU+GPS"
    else:
        sub_title = "IMU+GPS+LIDAR"

    sensor_noise_levels = config_data.get('sensor_noise_levels', {})
    sensor_info_list = [f"{key.upper()}: {value}" for key,
                        value in sensor_noise_levels.items()]
    sensor_info_str = "\n".join(sensor_info_list)

    sensor_rate_list = [f"{key.upper()}: {value} " for key,
                        value in sensor_update_rates.items()]
    sensor_rate_str = "\n".join(sensor_rate_list)

    overlap = config_data.get('Overlap', False)
    colors = list(plt.rcParams['axes.prop_cycle'].by_key()['color'])

    return sub_title, sensor_info_str, sensor_rate_str, overlap, colors

def plot_figures(fig, axs, states, observabilities, titles, sub_title, colors):
    plot_radar_charts(axs, states, observabilities, titles, colors)
    plt.figtext(0.5, 0.85, "Observability and State Mapping",
                ha="center", va="center", fontsize=22, color='blue')
    plt.figtext(0.5, 0.8, sub_title, ha="center",
                va="center", fontsize=18, color='blue')

def save_plots(config_data):
    save_directory = f"img/{config_data.get('motion_state', 'N/A')}/"
    if not os.path.exists(save_directory):
        os.makedirs(save_directory)
    save_path = os.path.join(
        save_directory, f"{config_data.get('motion_state', 'N/A')}" + "_states.png")
    plt.savefig(save_path)
    # save_svg_path = os.path.join(
    #     save_directory, f"{config_data.get('motion_state', 'N/A')}" + "_states.pdf")
    # plt.savefig(save_svg_path, dpi=500)

def setup_and_save_plots(config_data, states, observabilities, titles, sensor_update_rates):
    if not states:
        print("did not find any data to plot")
        return

    sub_title, sensor_info_str, sensor_rate_str, overlap, colors = prepare_plot_elements(
        config_data, sensor_update_rates)

    num_data = len(states)
    rows, cols = get_subplots_layout(num_data)

    fig, axs = plt.subplots(rows, cols, subplot_kw={'projection': 'polar'}, figsize=(25.4, 7.3), gridspec_kw={'wspace': 0.3, 'hspace': 0.3}) 
    axs = axs.flatten().tolist() if rows > 1 or cols > 1 else [axs]

    colors_cycle = iter(colors)

    for ax, state, observability, title in zip(axs, states, observabilities, titles):
        color = next(colors_cycle)
        plot_single_chart(ax, state, observability, title, color)

    for i in range(num_data, rows * cols):
        axs[i].axis('off')

    plt.subplots_adjust(left=0.03, right=0.97, top=1.14, bottom=0.01, wspace=0.1, hspace=0.1)
    save_plots(config_data)



def main():

    config_filepath = 'config/config.yaml'
    config_data = load_yaml_data(config_filepath)

    states, observabilities, titles, sensor_update_rates = load_data(config_data)
    setup_and_save_plots(config_data, states, observabilities, titles,sensor_update_rates)

    plt.gcf().canvas.mpl_connect('key_press_event', on_key)
    plt.show()

if __name__ == "__main__":
    main()

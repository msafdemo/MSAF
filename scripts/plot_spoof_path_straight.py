# /usr/bin/python3.8
import os
import matplotlib.pyplot as plt
import yaml
import re

red_legend_added = False
spoof_dev_legend_added = False
legend_handles_labels = []

def on_key(event):
    plt.close()

def custom_sort_key(label):
    special_labels_order = {
        'Exp. Dev.': (0,), 
        'Spoofing Point': (1,),  
    }

    if label in special_labels_order:
        return special_labels_order[label]

    numbers = re.findall(r'\d+\.?\d*', label)
    if numbers:
        return (2, float(numbers[0]))
    else:
        return (3, label.lower())

def get_csv_filenames(folder_path):
    all_files = os.listdir(folder_path)
    csv_files = [os.path.splitext(file)[0] for file in all_files if file.endswith('.csv')]
    return csv_files

def plot_data_points(subfolder, times, x_positions, y_positions, spoofing_time, spoofing_count):
    global red_legend_added, legend_handles_labels
    red_count = 0
    red_y_max = None
    x_max_at_red_ymax = None

    line, = plt.plot(x_positions, y_positions, label=subfolder, linewidth=3, zorder=1) 
    line_color = line.get_color()
    legend_handles_labels.append((line, subfolder, y_positions[-1] if y_positions else float('-inf')))
    
    for time, x, y in zip(times, x_positions, y_positions):
        if time >= spoofing_time and red_count < spoofing_count:
            label = 'Spoofing Point' if not red_legend_added else None
            handle = plt.scatter(x, y, color='red', s=90, label=label, zorder=2) 
            if label:
                legend_handles_labels.append((handle, label, float('-inf'))) 

            red_count += 1
            red_legend_added = True
            if red_y_max is None or y > red_y_max:
                red_y_max = y
                x_max_at_red_ymax = x

    return red_y_max, x_max_at_red_ymax, line_color


def plot_deviation_and_percentage(red_y_max, x_max_at_red_ymax, expected_deviation, line_color, subfolder, show_percentage):
    global spoof_dev_legend_added, legend_handles_labels

    label = 'Exp. Dev.' if not spoof_dev_legend_added else None
    handle = plt.axhline(y=expected_deviation, color='purple', linestyle='--', label=label, linewidth=3)
    if label:
        legend_handles_labels.append((handle, label, expected_deviation))
    spoof_dev_legend_added = True
    
    if red_y_max is not None:
        hline_left = x_max_at_red_ymax - 10     
        hline_right = x_max_at_red_ymax + 10

        diff = abs(red_y_max - expected_deviation)
        percent_diff = (diff / expected_deviation) * 100
        sign = "  +" if red_y_max > expected_deviation else "  -"
        
        if show_percentage and percent_diff >= 0.1:
            plt.annotate(f"{sign}{percent_diff:.2f}%", xy=(x_max_at_red_ymax, (red_y_max + expected_deviation) / 2), textcoords="data", ha="left", va="center", fontsize=18)
            arrowprops = {'arrowstyle': '<->', 'lw': 1.0, 'color': 'black'}
            plt.annotate("", xy=(x_max_at_red_ymax, red_y_max), xytext=(x_max_at_red_ymax, expected_deviation), arrowprops=arrowprops)


def Handle_subfolder(subfolder, motion_state, spoofing_time, spoofing_count, expected_deviation, show_percentage):
    subfolder_path = os.path.join("data/sim_data", motion_state, subfolder, "spoof_pose.txt")

    if os.path.exists(subfolder_path):
        with open(subfolder_path, 'r') as txt_file:
            lines = txt_file.readlines()
            times, y_positions, x_positions = [], [], []  
            
            for line in lines:
                time, y, x = map(float, line.split(','))  
                times.append(time)
                y_positions.append(y)
                x_positions.append(x)
            
            red_y_max, x_max_at_red_ymax, line_color = plot_data_points(subfolder, times, x_positions, y_positions, spoofing_time, spoofing_count)
            plot_deviation_and_percentage(red_y_max, x_max_at_red_ymax, expected_deviation, line_color, subfolder, show_percentage)

        # read and plot ref_data_posi.txt
        gt_path = os.path.join("data/sim_data", motion_state, subfolder, "ref_data_posi.txt")
        if os.path.exists(gt_path):
            with open(gt_path, 'r') as txt_file:
                lines = txt_file.readlines()
                gt_times, gt_y_positions, gt_x_positions = [], [], []  
                
                for line in lines:
                    time, y, x = map(float, line.split(',')) 
                    gt_times.append(time)
                    gt_y_positions.append(y)
                    gt_x_positions.append(x)

                plt.plot(gt_x_positions, gt_y_positions, color='grey', label=f"GT for {subfolder}")


        return y_positions[-1] if y_positions else float('-inf')
    else:
        print(f"File not found for {subfolder}: {subfolder_path}")
        return float('-inf')

def load_config(config_file):
    with open(config_file, 'r') as yaml_file:
        config = yaml.safe_load(yaml_file)
    return config

def Handle_and_plot_data(motion_states, motion_state, spoofing_time, spoofing_count, expected_deviation, show_percentage):
    last_y_values = {}
    for subfolder in motion_states:
        last_y_values[subfolder] = Handle_subfolder(subfolder, motion_state, spoofing_time, spoofing_count, expected_deviation, show_percentage)
    return last_y_values

def save_and_show_plot(motion_state, last_y_values):
    global legend_handles_labels

    # sort by y position first, then by label
    legend_handles_labels = sorted(legend_handles_labels, key=lambda x: custom_sort_key(x[1]))

    handles, labels = zip(*[(entry[0], entry[1]) for entry in legend_handles_labels])
    plt.legend(handles, labels, fontsize=40)

    plt.tick_params(axis='both', which='major', labelsize=48) 
    plt.xlabel("Y Coordinate(m)", fontsize=48, labelpad=20)
    plt.ylabel("X Coordinate(m)", fontsize=48)

    save_directory = f"img/{motion_state}/"

    if not os.path.exists(save_directory):
        os.makedirs(save_directory)

    save_path = os.path.join(save_directory, f"{motion_state}" + "_spoofed.png")
    plt.savefig(save_path)
    # save_path_svg = os.path.join(save_directory, f"{motion_state}" + "_spoofed.svg")
    # plt.savefig(save_path_svg, dpi=500)

    plt.gcf().canvas.mpl_connect('key_press_event', on_key)
    plt.show()


def main():
    global legend_handles_labels

    config_file = "config/config.yaml"
    config = load_config(config_file)

    motion_state = config.get('motion_state', 'straight_vel')
    spoofing_time = config.get("spoofing_time", 0)
    spoofing_count = config.get("spoofing_count", 0)
    expected_deviation = config.get("expected_deviation_for_straight", 0)
    show_percentage = config.get("show_percentage", True)
    motion_def_folder = f"src/motion_data_generator/motion_files/{motion_state}"
    motion_states = get_csv_filenames(motion_def_folder)

    plt.figure(figsize=(24, 16))

    last_y_values = Handle_and_plot_data(motion_states, motion_state, spoofing_time, spoofing_count, expected_deviation, show_percentage)
    save_and_show_plot(motion_state, last_y_values)


if __name__ == "__main__":
    main()

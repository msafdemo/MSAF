# /usr/bin/python3.8
import os
import matplotlib.pyplot as plt
import yaml
import re
from math import sqrt

red_legend_added = False
spoof_dev_legend_added = False

legend_handles_labels = []

def on_key(event):
    plt.close()

def custom_sort_key(label):
    pattern = r'(\d+\.?\d*|[a-zA-Z]+)'
    parts = re.findall(pattern, label)
    
    sorted_parts = []
    for part in parts:
        if part.replace('.', '', 1).isdigit():
            sorted_parts.append((1, float(part)))
        else:
            sorted_parts.append((0, part.lower()))
    return sorted_parts


def get_csv_filenames(folder_path):
    all_files = os.listdir(folder_path)
    csv_files = [os.path.splitext(file)[0] for file in all_files if file.endswith('.csv')]
    return csv_files

def plot_data_points(subfolder, times, x_positions, y_positions):
    line, = plt.plot(x_positions, y_positions, label=subfolder, linewidth=3)
    line_color = line.get_color()
    legend_handles_labels.append((line, subfolder, y_positions[-1] if y_positions else float('-inf')))
    return line_color 

def plot_deviation_and_percentage(red_y_max, x_max_at_red_ymax, spoofing_dev, line_color, subfolder):
    global spoof_dev_legend_added, legend_handles_labels

    label = 'Exp. Dev.' if not spoof_dev_legend_added else None

    spoof_dev_legend_added = True
    
    if red_y_max is not None:


        diff = abs(red_y_max - spoofing_dev)
        percent_diff = (diff / spoofing_dev) * 100
        sign = "  +" if red_y_max > spoofing_dev else "  -"
        
        if percent_diff >= 5:
            plt.annotate(f"{sign}{percent_diff:.2f}%", xy=(x_max_at_red_ymax, (red_y_max + spoofing_dev) / 2), textcoords="data", ha="left", va="center", fontsize=18)
            arrowprops = {'arrowstyle': '<->', 'lw': 1.0, 'color': 'black'}
            plt.annotate("", xy=(x_max_at_red_ymax, red_y_max), xytext=(x_max_at_red_ymax, spoofing_dev), arrowprops=arrowprops)

dashed_legend_added = {} 

dashed_line_points = []

def plot_parallel_dashed_lines(gt_x_positions, gt_y_positions, expected_deviation, line_color, subfolder):
    global dashed_line_points
    global dashed_legend_added
    dashed_line_points.clear()

    if subfolder not in dashed_legend_added:
        label = f'Exp. Dev. for {subfolder}'
        dashed_legend_added[subfolder] = True
    else:
        label = None

    for i in range(1, len(gt_x_positions)):
        x1, y1 = gt_x_positions[i-1], gt_y_positions[i-1]
        x2, y2 = gt_x_positions[i], gt_y_positions[i]

        # calculate direction vector
        dx = x2 - x1
        dy = y2 - y1

        # calculate length of direction vector
        length = sqrt(dx ** 2 + dy ** 2)
        unit_dx = dx / length
        unit_dy = dy / length

        # calculate new points for the parallel lines
        new_x1 = x1 + expected_deviation * unit_dy
        new_y1 = y1 - expected_deviation * unit_dx
        new_x2 = x2 + expected_deviation * unit_dy
        new_y2 = y2 - expected_deviation * unit_dx

        # draw dashed line with specified color as purple
        line, = plt.plot([new_x1, new_x2], [new_y1, new_y2], linestyle='--', color='purple', label=label, linewidth=3)

        # only want to show label once in the legend, so check if label is not None
        if label is not None:
            legend_handles_labels.append((line, label))
            label = None  # Prevent adding the label again in subsequent iterations


def Handle_subfolder(subfolder, motion_state, expected_deviation, spoofing_count, threshold):
    subfolder_path = os.path.join("data/sim_data", motion_state, subfolder, "spoof_pose.txt")
    gt_path = os.path.join("data/sim_data", motion_state, subfolder, "ref_data_posi.txt")
    
    line_color = None 
    all_spoofing_points = []
    distances_to_gt = []
    anomaly_detected = False
    percentage_distances_to_gt = []
    
    if os.path.exists(subfolder_path) and os.path.exists(gt_path):
        with open(subfolder_path, 'r') as txt_file:
            lines = txt_file.readlines()
            times, y_positions, x_positions = [], [], []
            for line in lines:
                time, y, x = map(float, line.split(','))
                times.append(time)
                y_positions.append(y)
                x_positions.append(x)

            line_color = plot_data_points(subfolder, times, x_positions, y_positions) 
        
        with open(gt_path, 'r') as txt_file:
            lines = txt_file.readlines()
            gt_times, gt_y_positions, gt_x_positions = [], [], []
            for line in lines:
                time, y, x = map(float, line.split(','))
                gt_times.append(time)
                gt_y_positions.append(y)
                gt_x_positions.append(x)

            # label anomaly points
            for i in range(min(len(x_positions), len(gt_x_positions))):
                if abs(x_positions[i] - gt_x_positions[i]) > threshold or abs(y_positions[i] - gt_y_positions[i]) > threshold:
                    if not anomaly_detected: 
                        all_spoofing_points.append((x_positions[i], y_positions[i])) 
                        anomaly_detected = True 
                        
                        # calculate distance from point to line
                        if i > 0: 
                            distance = point_to_line_distance(
                                x_positions[i], y_positions[i],
                                gt_x_positions[i-1], gt_y_positions[i-1],
                                gt_x_positions[i], gt_y_positions[i]
                            )
                            distances_to_gt.append(distance)
                            percentage_distance = (distance / expected_deviation) * 100-100
                            percentage_distances_to_gt.append(percentage_distance)
                            
                    plt.plot(x_positions[i], y_positions[i], 'ro', markersize=6)
                else:
                    anomaly_detected = False
        print(f"Percentage distances to ref_data_posi for {subfolder}: {percentage_distances_to_gt}")
        return y_positions[-1] if y_positions else float('-inf'), line_color, all_spoofing_points, percentage_distances_to_gt 
    else:
        print(f"File not found for {subfolder}: {subfolder_path}")
        return float('-inf'), None, [], []

def Handle_and_plot_data(motion_states, motion_state, expected_deviation, spoofing_count, threshold, show_percentage, max_x_offset=1.8, max_y_offset=0, min_x_offset=0.1, min_y_offset=0):
    last_y_values = {}
    color_dict = {}
    
    all_spoofing_points_dict = {}
    all_percentage_distances_dict = {}
    max_percentage_subfolder = None
    max_percentage_value = -1

    min_percentage_subfolder = None
    min_percentage_value = float('inf')

    for subfolder in motion_states:
        last_y_value, color, all_spoofing_points, percentage_distances_to_gt = Handle_subfolder(subfolder, motion_state, expected_deviation, spoofing_count, threshold)
        
        if all_spoofing_points: 
            all_spoofing_points_dict[subfolder] = all_spoofing_points
            all_percentage_distances_dict[subfolder] = percentage_distances_to_gt
            
            max_value_in_subfolder = max(percentage_distances_to_gt) if percentage_distances_to_gt else -1
            if max_value_in_subfolder > max_percentage_value:
                max_percentage_value = max_value_in_subfolder
                max_percentage_subfolder = subfolder
            
            min_value_in_subfolder = min(percentage_distances_to_gt) if percentage_distances_to_gt else float('inf') 
            if min_value_in_subfolder < min_percentage_value:
                min_percentage_value = min_value_in_subfolder
                min_percentage_subfolder = subfolder
            
        last_y_values[subfolder] = last_y_value
        if color:
            color_dict[subfolder] = color

        subfolder_path = os.path.join("data/sim_data", motion_state, subfolder, "ref_data_posi.txt")
        if os.path.exists(subfolder_path):
            with open(subfolder_path, 'r') as txt_file:
                lines = txt_file.readlines()
                gt_times, gt_y_positions, gt_x_positions = [], [], []
                for line in lines:
                    time, y, x = map(float, line.split(','))
                    gt_times.append(time)
                    gt_y_positions.append(y)
                    gt_x_positions.append(x)

                if color:
                    plot_parallel_dashed_lines(gt_x_positions, gt_y_positions, expected_deviation, color, subfolder)

    # show percentage annotations (add trajectories for max and min percentage distances)
    if show_percentage:
        for percentage_subfolder, percentage_value, x_offset, y_offset in [
            (max_percentage_subfolder, max_percentage_value, max_x_offset, max_y_offset), 
            (min_percentage_subfolder, min_percentage_value, min_x_offset, min_y_offset)
        ]:
            if percentage_subfolder:
                points = all_spoofing_points_dict.get(percentage_subfolder, [])
                percentages = all_percentage_distances_dict.get(percentage_subfolder, [])
                
                for i in range(len(points)):
                    point = points[i]
                    percentage = percentages[i] if i < len(percentages) else 0
                    sign = "+" if percentage >= 0 else "-"
                    plt.annotate(
                        f"{sign}{abs(percentage):.2f}%", 
                        xy=(point[0], point[1]), 
                        xytext=(point[0] + x_offset, point[1] + y_offset),
                        textcoords="data", 
                        ha="right", 
                        va="bottom"
                    )
    
    return last_y_values

def load_config(config_file):
    with open(config_file, 'r') as yaml_file:
        config = yaml.safe_load(yaml_file)
    return config

# add a red dot to the plot as a special legend entry
def add_spoofing_point_legend():
    global legend_handles_labels

    red_dot, = plt.plot([], [], 'ro', markersize=8, label='Spoofing Point')     # 图例红点
    legend_handles_labels = [(red_dot, 'Spoofing Point')] + legend_handles_labels

def save_and_show_plot(motion_state, last_y_values):
    global legend_handles_labels

    # sorted by custom key
    other_legend = sorted([entry for entry in legend_handles_labels if entry[1] != 'Spoofing Point'], key=lambda x: custom_sort_key(x[1]))

    # make sure 'Spoofing Point' is always the first entry
    final_legend = [entry for entry in legend_handles_labels if entry[1] == 'Spoofing Point'] + other_legend

    handles, labels = zip(*[(entry[0], entry[1]) for entry in final_legend])
    if motion_state == 'turning_yaw_vel':
        plt.legend(handles, labels, fontsize=26)
    elif motion_state == 'turning_yaw':
        plt.legend(handles, labels, fontsize=14)
    

    # plt.title(f"Spoofed Positions for {motion_state}")
    plt.xlabel("Y Coordinate(m)", fontsize=30, labelpad=20)
    plt.ylabel("X Coordinate(m)", fontsize=30)
    plt.tick_params(axis='both', which='major', labelsize=30)  
    save_directory = f"img/{motion_state}/"

    if not os.path.exists(save_directory):
        os.makedirs(save_directory)

    save_path_png = os.path.join(save_directory, f"{motion_state}" + "_spoofed.png")
    plt.savefig(save_path_png, dpi=500)

    # save_path_svg = os.path.join(save_directory, f"{motion_state}" + "_spoofed.svg")
    # plt.savefig(save_path_svg, dpi=500)

    plt.gcf().canvas.mpl_connect('key_press_event', on_key)
    plt.show()

def point_to_line_distance(x, y, x1, y1, x2, y2):
    """
    Calculate the distance from a point (x, y) to a line passing through points (x1, y1) and (x2, y2).
    """
    if x1 == x2 and y1 == y2:
        raise ValueError("The two points cannot be identical.")
    
    m = (y2 - y1) / (x2 - x1)  # Slope of the line
    A = -m
    B = 1
    C = m * x1 - y1

    distance = abs(A * x + B * y + C) / sqrt(A ** 2 + B ** 2)
    return distance

def calculate_percentage_distances(distances_to_gt, expected_deviation):
    percentage_distances_to_gt = []
    for distance in distances_to_gt:
        percentage_distance = (distance / expected_deviation) * 100
        percentage_distances_to_gt.append(percentage_distance)
    return percentage_distances_to_gt


def main():
    global legend_handles_labels

    config_file = "config/config.yaml"
    config = load_config(config_file)

    motion_state = config.get('motion_state', 'straight_vel')
    expected_deviation = config.get("expected_deviation_for_turning", 0)
    spoofing_count = config.get("spoofing_count", 1) 
    threshold = config.get("filter_threshold", 0.5) 
    show_percentage = config.get('show_percentage', True)

    motion_def_folder = f"src/motion_data_generator/motion_files/{motion_state}"
    motion_states = get_csv_filenames(motion_def_folder)
    if motion_state == 'turning_yaw':
        plt.figure(figsize=(24, 16))
    elif motion_state == 'turning_yaw_vel':
        plt.figure(figsize=(16, 24))
    add_spoofing_point_legend()

    last_y_values = Handle_and_plot_data(motion_states, motion_state, expected_deviation, spoofing_count, threshold, show_percentage)
    save_and_show_plot(motion_state, last_y_values)

if __name__ == "__main__":
    main()
import matplotlib.pyplot as plt
import numpy as np
import yaml
import os
import re

def on_key(event):
    plt.close()

def read_matrices_from_file(file_path):
    k_matrices = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        matrix = []
        for line in lines:
            if 'Matrix:' in line:
                if matrix:
                    k_matrices.append(np.array(matrix))
                matrix = []
            else:
                row = list(map(float, line.strip().split()))
                matrix.append(row)
        if matrix:
            k_matrices.append(np.array(matrix))
    return k_matrices

def analyze_k_matrices(k_matrices, label_suffix='', key_index=(0, 0)):
    k_values = [K[key_index] for K in k_matrices]
    time_steps = np.linspace(0, 20, len(k_values))
    clean_label_suffix = label_suffix.replace("K_for_", "")
    label = f'$K_{{({key_index[0]+1},{key_index[1]+1})}}$ for {clean_label_suffix}'
    plt.plot(time_steps, k_values, label=label, marker='o', markersize=3, linewidth=2)

def natural_keys(text):
    return [float(c) if c.replace('.', '', 1).isdigit() else c for c in re.split('([0-9.]+)', text)]

def Handle_k_matrices(base_path, key_index):
    file_names = [f for f in os.listdir(base_path) if os.path.isfile(os.path.join(base_path, f))]
    file_names = sorted(file_names, key=natural_keys)

    for file in file_names:
        file_path = os.path.join(base_path, file)
        k_matrices = read_matrices_from_file(file_path)
        label_suffix = os.path.splitext(file)[0]
        analyze_k_matrices(k_matrices, label_suffix=label_suffix, key_index=key_index)

def save_plots(motion_state):
    save_directory = f"img/{motion_state}/"
    if not os.path.exists(save_directory):
        os.makedirs(save_directory)
    save_path = os.path.join(save_directory, f"{motion_state}_K_values.png")
    plt.savefig(save_path, dpi=500)
    # sva_svg_path = os.path.join(save_directory, f"{motion_state}_K_values.svg")
    # plt.savefig(sva_svg_path, dpi=500)

def main():
    with open("config/config.yaml", 'r') as yamlfile:
        cfg = yaml.safe_load(yamlfile)
        motion_state = cfg['motion_state']

    base_path = f"data/K_values/{motion_state}/"
    print("Debug: base_path:", base_path)

    plt.figure(figsize=(15, 8))
    
    Handle_k_matrices(base_path, key_index=(0, 9))
    # Handle_k_matrices(base_path, key_index=(0,0))

    plt.tick_params(axis='both', which='major', labelsize=36)
    # plt.title('Value Changes of K11 Element in K Matrices')
    plt.xlabel('Time (seconds)', fontsize=36, labelpad=20) 
    plt.ylabel('Kalman Gain', fontsize=36)
    plt.legend(fontsize=30, framealpha=1.0)
    plt.grid(True)
    
    plt.tight_layout()
    plt.subplots_adjust(left=0.1)
    
    save_plots(motion_state)
    plt.gcf().canvas.mpl_connect('key_press_event', on_key)
    plt.show()

if __name__ == "__main__":
    main()

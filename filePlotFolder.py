# Imports
import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np
import argparse
import os

def plot_imu_data(filename, save_path):
    headers, values = FileReader(filename).read_file()
    time_list = []
    first_stamp = values[0][-1]

    for val in values:
        time_list.append((val[-1] - first_stamp) * 1e-9)

    plt.figure()
    for i in range(len(headers) - 1):
        plt.plot(time_list, [float(lin[i]) for lin in values], label=headers[i])
    
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2); Angular Velocity (rad/s)")
    plt.title(f"IMU Data from {filename}")
    plt.legend()
    plt.grid()

    plt.savefig(save_path)
    plt.close()

def plot_laser_data(filename, save_path):
    ranges, angle_increment, timestamps = FileReader(filename).read_laser_file()
    
    is_inf = np.isinf(ranges)[0]
    x = []
    y = []
    range_list = ranges[0]
    theta = 0
    for i, dist in enumerate(range_list):
        if not is_inf[i]:
            x.append(dist * np.cos(theta))
            y.append(dist * np.sin(theta))
        theta += angle_increment

    plt.scatter(x, y)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title(f'LiDAR Ranges from {filename}')

    plt.savefig(save_path)
    plt.close()

def plot_odom_data(filename, save_path):
    headers, values = FileReader(filename).read_file()
    time_list = []
    first_stamp = values[0][-1]

    for val in values:
        time_list.append((val[-1] - first_stamp) * 1e-9)

    plt.figure()
    for i in range(len(headers) - 1):
        plt.plot(time_list, [float(lin[i]) for lin in values], label=headers[i])

    plt.xlabel("Time (s)")
    plt.ylabel("Euler Angles")
    plt.title(f"Odom Data from {filename}")
    plt.legend()
    plt.grid()

    plt.savefig(save_path)
    plt.close()

def plot_file(filename, output_folder):
    base_filename = os.path.basename(filename)
    output_path = os.path.join(output_folder, base_filename.replace('.csv', '.png'))

    if 'imu_content' in filename:
        plot_imu_data(filename, output_path)
    elif 'laser_content' in filename:
        plot_laser_data(filename, output_path)
    elif 'odom_content' in filename:
        plot_odom_data(filename, output_path)
    else:
        print(f"Unknown file type for {filename}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process CSV files in a folder and save plots as PNG.')
    parser.add_argument('--folder', required=True, help='Folder containing the CSV files')
    parser.add_argument('--output', required=True, help='Folder to save the PNG files')

    args = parser.parse_args()

    input_folder = args.folder
    output_folder = args.output

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    csv_files = [os.path.join(input_folder, f) for f in os.listdir(input_folder) if f.endswith('.csv')]

    for filename in csv_files:
        plot_file(filename, output_folder)

    print(f"Plots saved to {output_folder}")

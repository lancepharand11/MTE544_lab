# Imports
import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np
import argparse

def plot_imu_data(filename):
    headers, values = FileReader(filename).read_file()
    time_list = []
    first_stamp = values[0][-1]

    for val in values:
        time_list.append((val[-1] - first_stamp) * 1e-9)  # Convert to seconds for readability

    plt.figure()
    for i in range(len(headers) - 1):  # Exclude the timestamp from headers
        plt.plot(time_list, [float(lin[i]) for lin in values], label=headers[i])
    
    plt.xlabel("Time (s)")
    plt.ylabel("IMU Data")
    plt.title(f"IMU Data from {filename}")
    plt.legend()
    plt.grid()
    plt.show()


def plot_laser_data(filename):
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
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Ranges')

    plt.show()


def plot_odom_data(filename):
    headers, values = FileReader(filename).read_file()
    time_list = []
    first_stamp = values[0][-1]

    for val in values:
        time_list.append((val[-1] - first_stamp) * 1e-9)  # Convert to seconds

    plt.figure()
    for i in range(len(headers) - 1):  # Exclude the timestamp from headers
        plt.plot(time_list, [float(lin[i]) for lin in values], label=headers[i])

    plt.xlabel("Time (s)")
    plt.ylabel("Odom Data")
    plt.title(f"Odom Data from {filename}")
    plt.legend()
    plt.grid()
    plt.show()


def plot_file(filename):
    if 'imu_content' in filename:
        plot_imu_data(filename)
    elif 'laser_content' in filename:
        plot_laser_data(filename)
    elif 'odom_content' in filename:
        plot_odom_data(filename)
    else:
        print(f"Unknown file type for {filename}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')

    args = parser.parse_args()

    print("Plotting the files:", args.files)

    filenames = args.files
    for filename in filenames:
        plot_file(filename)

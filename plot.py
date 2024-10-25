import pandas as pd
import matplotlib.pyplot as plt
import os

# Load data from CSV files
def load_data(folder):
    robot_pose_df = pd.read_csv(os.path.join(folder, "robot_pose.csv"))
    angular_df = pd.read_csv(os.path.join(folder, "angular.csv"))
    linear_df = pd.read_csv(os.path.join(folder, "linear.csv"))
    return robot_pose_df, angular_df, linear_df

# Ensure that the directory exists for saving images
def ensure_dir(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

# Plot {x-t}, {y-t}, {theta-t}, {e-t}, {edot-t}, {x-y}, and {e-edot}
def plot_robot_trajectories(robot_pose_df, angular_df, linear_df, folder_name):
    # Create a directory for saving plots under img/{folder_name}/
    img_dir = f'img/{folder_name}/'
    ensure_dir(img_dir)

    # Zero out the time axis by subtracting the first timestamp
    time_robot = (robot_pose_df[' stamp'] - robot_pose_df[' stamp'][0]) / 1e9  # Convert nanoseconds to seconds for plotting
    time_error = (angular_df[' stamp'] - angular_df[' stamp'][0]) / 1e9  # Convert nanoseconds to seconds for plotting

    # {x-t}, {y-t}, {theta-t} plots
    plt.figure()
    plt.plot(time_robot, robot_pose_df['x'], label='x-t')
    plt.plot(time_robot, robot_pose_df[' y'], label='y-t')
    plt.plot(time_robot, robot_pose_df[' theta'], label='theta-t')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m) / Angle (rad)')  # Assuming x, y are in meters
    plt.title(f'Robot Pose vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'{img_dir}position_vs_time.png')

    # {x-y} plot
    plt.figure()
    plt.plot(robot_pose_df['x'], robot_pose_df[' y'], label='x-y')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title(f'x-y Trajectory')
    plt.grid(True)
    plt.savefig(f'{img_dir}xy_trajectory.png')

    # {e-t} and {edot-t} plots combined in one figure but separated into linear and angular subplots
    fig, axs = plt.subplots(2, 1, figsize=(8, 8))

    # Angular error and error dot vs time
    axs[0].plot(time_error, angular_df['e'], label='Angular Error (e-t)')
    axs[0].plot(time_error, angular_df[' e_dot'], label='Angular Error Dot (edot-t)')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Angle (rad) / Angular Velocity (rad/s)')  # Assuming angular error in radians and error dot in rad/s
    axs[0].set_title('Angular Error and Error Dot vs Time')
    axs[0].legend()
    axs[0].grid(True)

    # Linear error and error dot vs time
    axs[1].plot(time_error, linear_df['e'], label='Linear Error (e-t)')
    axs[1].plot(time_error, linear_df[' e_dot'], label='Linear Error Dot (edot-t)')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Distance (m) / Velocity (m/s)')  # Assuming linear error in meters and error dot in m/s
    axs[1].set_title('Linear Error and Error Dot vs Time')
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    plt.savefig(f'{img_dir}error_and_edot_vs_time.png')

    # {e-edot} plot (linear and angular separated in the same figure, side by side)
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))  # Adjusted to make the plots side by side

    # Angular error vs error dot
    axs[0].plot(angular_df['e'], angular_df[' e_dot'], label='Angular Error vs Error Dot')
    axs[0].set_xlabel('Angle (rad)')
    axs[0].set_ylabel('Angular Velocity (rad/s)')
    axs[0].set_title('Angular Error vs Error Dot')
    axs[0].legend()
    axs[0].grid(True)

    # Linear error vs error dot
    axs[1].plot(linear_df['e'], linear_df[' e_dot'], label='Linear Error vs Error Dot')
    axs[1].set_xlabel('Distance (m)')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].set_title('Linear Error vs Error Dot')
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    plt.savefig(f'{img_dir}error_vs_error_dot.png')

# Main function to handle plotting for both controllers
def main():
    # Folders for each controller
    folders = ["parabola_pid", "point_p", "point_pid", "sigmoid_pid"]

    for folder in folders:
        if os.path.exists(folder):
            robot_pose_df, angular_df, linear_df = load_data(folder)
            plot_robot_trajectories(robot_pose_df, angular_df, linear_df, folder)
        else:
            print(f"Folder {folder} does not exist")

if __name__ == "__main__":
    main()

import pandas as pd
import matplotlib.pyplot as plt
import os

# Load data from CSV files
def load_data(folder):
    robot_pose_df = pd.read_csv(os.path.join(folder, "robot_pose.csv"))
    angular_df = pd.read_csv(os.path.join(folder, "angular.csv"))
    linear_df = pd.read_csv(os.path.join(folder, "linear.csv"))
    return robot_pose_df, angular_df, linear_df

# Plot {x-t}, {y-t}, {theta-t}, {e-t}, {edot-t}, {x-y}, and {e-edot}
def plot_robot_trajectories(robot_pose_df, angular_df, linear_df, folder_name):
    time_robot = robot_pose_df['stamp'] / 1e9  # Convert nanoseconds to seconds for plotting

    # {x-t}, {y-t}, {theta-t} plots
    plt.figure()
    plt.plot(time_robot, robot_pose_df['x'], label='x-t')
    plt.plot(time_robot, robot_pose_df['y'], label='y-t')
    plt.plot(time_robot, robot_pose_df['theta'], label='theta-t')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title(f'{folder_name}: Robot Position over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'{folder_name}_position_vs_time.png')

    # {x-y} plot
    plt.figure()
    plt.plot(robot_pose_df['x'], robot_pose_df['y'], label='x-y')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title(f'{folder_name}: x-y Trajectory')
    plt.grid(True)
    plt.savefig(f'{folder_name}_xy_trajectory.png')

    time_error = angular_df['Time.from_msg(stamp).nanoseconds'] / 1e9  # Convert nanoseconds to seconds for plotting

    # {e-t} plot (error vs time)
    plt.figure()
    plt.plot(time_error, angular_df['latest_error'], label='Angular Error (e-t)')
    plt.plot(time_error, linear_df['latest_error'], label='Linear Error (e-t)')
    plt.xlabel('Time (s)')
    plt.ylabel('Error')
    plt.title(f'{folder_name}: Error vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'{folder_name}_error_vs_time.png')

    # {edot-t} plot (error dot vs time)
    plt.figure()
    plt.plot(time_error, angular_df['error_dot'], label='Angular Error Dot (edot-t)')
    plt.plot(time_error, linear_df['error_dot'], label='Linear Error Dot (edot-t)')
    plt.xlabel('Time (s)')
    plt.ylabel('Error Dot')
    plt.title(f'{folder_name}: Error Dot vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'{folder_name}_error_dot_vs_time.png')

    # {e-edot} plot (error vs error dot)
    plt.figure()
    plt.plot(angular_df['latest_error'], angular_df['error_dot'], label='Angular Error vs Error Dot')
    plt.plot(linear_df['latest_error'], linear_df['error_dot'], label='Linear Error vs Error Dot')
    plt.xlabel('Error')
    plt.ylabel('Error Dot')
    plt.title(f'{folder_name}: Error vs Error Dot')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'{folder_name}_error_vs_error_dot.png')

# Main function to handle plotting for both controllers
def main():
    # Folders for each controller
    folders = ["parabola_pid", "point_p"]

    for folder in folders:
        if os.path.exists(folder):
            robot_pose_df, angular_df, linear_df = load_data(folder)
            plot_robot_trajectories(robot_pose_df, angular_df, linear_df, folder)
        else:
            print(f"Folder {folder} does not exist")

if __name__ == "__main__":
    main()

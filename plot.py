import os
import pandas as pd
import matplotlib.pyplot as plt

def plot_experiment_data(file_path, output_path):
    # Read the CSV file
    data = pd.read_csv(file_path)
    data.columns = data.columns.str.strip()  # Strip whitespace from headers
    
    # Extract headers and data
    headers = data.columns
    t = data['stamp']
    t = t - t[0]  # Normalize time to start at 0
    
    # Define figure and axis layout with height ratios
    fig, axs = plt.subplots(
        2, 1, figsize=(14, 6), gridspec_kw={'height_ratios': [1, 2]}
    )
    fig.tight_layout(pad=4)
    
    # Plot the state space in the first subplot
    axs[0].plot(data['kf_x'], data['kf_y'], label='Trajectory')
    axs[0].set_title('Trajectory', fontsize=18)
    axs[0].set_xlabel('x (m)', fontsize=16)
    axs[0].set_ylabel('y (m)', fontsize=16)
    axs[0].grid(True)
    
    # Plot all individual states in the second subplot
    for column in headers:
        if column != 'stamp':
            axs[1].plot(t, data[column], label=column)
    
    axs[1].set_title('State Estimation Parameters', fontsize=18)
    axs[1].set_xlabel('Time (s)', fontsize=16)
    axs[1].set_ylabel('Magnitudes', fontsize=16)
    axs[1].grid(True)
    axs[1].legend(fontsize=12, loc='upper right')
    
    # Save plot
    output_file = os.path.join(output_path, os.path.basename(file_path).replace('.csv', '.png'))
    plt.savefig(output_file, dpi=300)
    plt.close()


def main(input_folder, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    # Walk through the directory structure
    for root, _, files in os.walk(input_folder):
        for file in files:
            if file.endswith('.csv'):
                file_path = os.path.join(root, file)
                experiment_output_path = os.path.join(output_folder, os.path.relpath(root, input_folder))
                
                if not os.path.exists(experiment_output_path):
                    os.makedirs(experiment_output_path)
                
                # Generate plot for the CSV file
                plot_experiment_data(file_path, experiment_output_path)

# Specify your input folder and output folder
def generate_input_output_folders(base_input, base_output):
    input_output_pairs = []

    # Traverse the base input directory
    for root, dirs, files in os.walk(base_input):
        for file in files:
            if file.endswith('.csv'):
                # Input folder for this file
                input_folder = os.path.dirname(os.path.join(root, file))
                
                # Corresponding output folder
                relative_path = os.path.relpath(input_folder, base_input)
                output_folder = os.path.join(base_output, relative_path)
                
                input_output_pairs.append((input_folder, output_folder))

    return input_output_pairs

# Specify the base directories
base_input_folder = "./spiral"
base_output_folder = "./img/spiral"

# Generate input-output folder pairs
folders = generate_input_output_folders(base_input_folder, base_output_folder)

# Example of how to loop over these folders and call `main`
for input_folder, output_folder in folders:
    main(input_folder, output_folder)

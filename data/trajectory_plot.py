import pandas as pd
import matplotlib.pyplot as plt

def load_trajectory_data(file_path):
    data = pd.read_csv(file_path)
    odom = data[data['type'] == 'odom']
    real = data[data['type'] == 'real']
    estimator = data[data['type'] == 'estimator']
    return odom, real, estimator

# Function to filter the estimated trajectory
def filter_estimated_trajectory(estimator, threshold_x=2, threshold_y=2):
    return estimator[(estimator['x'].abs() < threshold_x) & (estimator['y'].abs() < threshold_y)]

# Function to plot the trajectories
def plot_trajectory(ax, trajectory, title, color):
    ax.plot(trajectory['x'], trajectory['y'], color=color)
    ax.set_title(title)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_aspect('equal')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)


file_paths = ['trajectory.csv', 'trajectory_odom_with_noise.csv', 'trajectory_odom_with_noise2.csv']

# Create subplots for each file
fig, axs = plt.subplots(3, 3, figsize=(24, 24))

for i, file_path in enumerate(file_paths):
    odom, real, estimator = load_trajectory_data(file_path)
    estimator_filtered = filter_estimated_trajectory(estimator)

    # Plot odometry trajectory
    plot_trajectory(axs[i, 0], odom, f'Odometry Trajectory {i+1}', color='blue')

    # Plot estimated trajectory
    plot_trajectory(axs[i, 1], estimator_filtered, f'Estimated Trajectory {i+1}', color='green')

    # Plot real trajectory
    plot_trajectory(axs[i, 2], real, f'Real Trajectory {i+1}', color='red')

plt.tight_layout()
plt.show()

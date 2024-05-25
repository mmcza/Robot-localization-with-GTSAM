
import pandas as pd
import matplotlib.pyplot as plt

file_path = 'trajectory.csv'
data = pd.read_csv(file_path)

odom = data[data['type'] == 'odom']
estimator = data[data['type'] == 'estimator']
real = data[data['type'] == 'real']

def plot_trajectory(ax, trajectory, title, color):
    ax.plot(trajectory['x'], trajectory['y'], color=color)
    ax.set_title(title)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_aspect('equal')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)

fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(24, 8))

# Plot odometry trajectory
plot_trajectory(ax1, odom, 'Odometry Trajectory', color='blue')

# Plot estimated trajectory without any filtering
plot_trajectory(ax2, estimator, 'Estimated Trajectory', color='green')

# Plot real trajectory
plot_trajectory(ax3, real, 'Real Trajectory', color='red')

plt.show()


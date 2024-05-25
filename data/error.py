import pandas as pd
import numpy as np

def load_trajectory_data(file_path):
    data = pd.read_csv(file_path)
    odom = data[data['type'] == 'odom']
    real = data[data['type'] == 'real']
    estimator = data[data['type'] == 'estimator']
    return odom, real, estimator

# Function to normalize the time to start from zero
def normalize_time(trajectory):
    trajectory['t'] = trajectory['t'] - trajectory['t'].iloc[0]
    return trajectory

# Function to filter the estimated trajectory
def filter_estimated_trajectory(estimator, threshold_x=2, threshold_y=2):
    return estimator[(estimator['x'].abs() < threshold_x) & (estimator['y'].abs() < threshold_y)]

# Function to calculate error
def calculate_error(estimated, real):
    error = np.sqrt((estimated['x'] - real['x'])**2 + (estimated['y'] - real['y'])**2)
    return error

file_path = 'trajectory_odom_with_noise.csv'

odom, real, estimator = load_trajectory_data(file_path)

odom = normalize_time(odom)
real = normalize_time(real)
estimator = normalize_time(estimator)

# Ensure the lengths match for comparison after normalizing time
min_length = min(len(real), len(estimator))
real = real.iloc[:min_length].reset_index(drop=True)
estimator = estimator.iloc[:min_length].reset_index(drop=True)

error = calculate_error(estimator, real)
print("First few errors:\n", error.head())
print("\nSummary statistics:\n", error.describe())
print("Last few errors:\n", error.tail())



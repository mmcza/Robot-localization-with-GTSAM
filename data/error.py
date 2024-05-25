import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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

# Function to calculate error
def calculate_error(estimated, real):
    error = np.sqrt((estimated['x'] - real['x'])**2 + (estimated['y'] - real['y'])**2)
    return error

def find_closest_timestamp(target, source):
    idx = np.searchsorted(source, target, side='left')
    idx = np.clip(idx, 1, len(source) - 1)
    left = source[idx - 1]
    right = source[idx]
    idx -= target - left < right - target
    return source[idx]

file_path = 'trajectory.csv'

odom, real, estimator = load_trajectory_data(file_path)

odom = normalize_time(odom)
real = normalize_time(real)
estimator = normalize_time(estimator)

closest_timestamps_o_r = find_closest_timestamp(real['t'].values, odom['t'].values)
closest_timestamps_e_r = find_closest_timestamp(real['t'].values, estimator['t'].values)

real['closest_timestamps_odom'] = closest_timestamps_o_r 
real['closest_timestamps_estimator'] = closest_timestamps_e_r 

def get_closest_row(df, timestamp):
    return df[df['t'] == timestamp].iloc[0]

# Calculate errors using apply and lambda
real['error_odom'] = real.apply(
    lambda row: calculate_error(get_closest_row(odom, row['closest_timestamps_odom']), row), axis=1)

real['error_estimator'] = real.apply(
    lambda row: calculate_error(get_closest_row(estimator, row['closest_timestamps_estimator']), row), axis=1)

real_filtered = real[real['t'] < 25]
real_filtered = real_filtered[real_filtered['t'] > 0]

print(real_filtered['error_estimator'].describe())
print(real_filtered['error_odom'].describe())

plt.figure(figsize=(12, 6))
plt.plot(real_filtered['t'], real_filtered['error_odom'], label='Error Odom', color='blue', marker='o')
plt.plot(real_filtered['t'], real_filtered['error_estimator'], label='Error Estimator', color='green', marker='x')

plt.xlabel('Time')
plt.ylabel('Error')
plt.title('Error Over Time')
plt.legend()
plt.grid(True)
plt.show()

print(real['error_estimator'])
print(real['error_odom'])

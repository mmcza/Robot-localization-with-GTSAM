import numpy as np

# Define the file name and read the data
filename = 'lidar_data.csv'
data = np.genfromtxt(filename, delimiter=',', skip_header=1, dtype=None, encoding=None)
parameter_var = np.var(data, axis=0)

# Display the data
print(parameter_var)
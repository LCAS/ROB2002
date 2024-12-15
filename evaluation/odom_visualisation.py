import matplotlib.pyplot as plt
import pandas as pd 

file_name = 'evaluation/data/odom_log.csv'

# use Pandas for convenient operations on tabular data
data = pd.read_csv(file_name, header=None)

# use matplotlib for visualisation
# visualise columns 5 and 6 corresponding to x,y coordinates of the odometry message
plt.plot(data.iloc[:, 5].to_numpy(),data.iloc[:, 6].to_numpy())

# adjust manually the plot limits, if required
# plt.axis([-1, 1, -1, 1]) # adjust the plot limits (for odometry message that is in meters!)

# show the plot: check it out in the no-vnc browser window!
plt.show()
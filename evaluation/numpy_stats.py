import csv
import numpy as np
import matplotlib.pyplot as plt

# load data from csv file as a numpy array
data = np.genfromtxt('evaluation/ref_data/head_size_gt.csv', delimiter=',')

# Broccoli head size measurements (in mm) for training vision systems for robotic harvesters
# for more details see Kusumam et al., 2017: https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21726

print('sample size {0:d}'.format(len(data)))

print('mean {0:.2f} std {1:.2f}'.format(np.mean(data), np.std(data)))
print('min {0:.2f} max {1:.2f} range {2:.2f}'.format(np.min(data), np.max(data), np.max(data) - np.min(data)))

print('median {0:.2f}'.format(np.median(data)))
print('qunatile: qnt_05% {0:.2f} qnt_95% {1:.2f} range {2:.2f}'.format(np.quantile(data,0.05), np.quantile(data,0.95),np.quantile(data,0.95)-np.quantile(data,0.05)))
print('quartile: lower {0:.2f} upper {1:.2f} range {2:.2f}'.format(np.quantile(data,0.25), 
                                                                                        np.quantile(data,0.75),np.quantile(data,0.75)-np.quantile(data,0.25)))

h_count, h_range = np.histogram(data, bins=20, range=(70,210))
print('histogram:')
print(' - bin counts ', h_count)
print(' - bin ranges ', h_range)

#plotting histograms with the same settings as above
plt.hist(data, bins=20, range=(60,220))
plt.show()
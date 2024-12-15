import csv
import numpy as np
import matplotlib.pyplot as plt

# load data from csv file as a numpy array
data = np.genfromtxt('evaluation/ref_data/head_size_gt.csv', delimiter=',')

# Broccoli head size measurements (in mm) for training vision systems for robotic harvesters
# for more details see Kusumam et al., 2017: https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21726

print('sample size {0:d}'.format(len(data)))

bin_nr = 20
bin_range = (70,210) # None for auto

# calculate all stats
st_mean = np.mean(data)
st_std = np.std(data)
st_min = np.min(data)
st_max = np.max(data)
st_median = np.median(data)
st_qnt_05 = np.quantile(data, 0.05)
st_qnt_95 = np.quantile(data, 0.95)
st_loqrt = np.quantile(data, 0.25)
st_upqrt = np.quantile(data, 0.75)
h_count, h_range = np.histogram(data, bins=bin_nr, range=bin_range)

# print out the results
print(f'mean {st_mean:.2f} std {st_std:.2f}')
print(f'min {st_min:.2f} max {st_max:.2f} range {st_max-st_min:.2f}')

print(f'median {st_median:.2f}')
print(f'quantile: qnt_05% {st_qnt_05:.2f} qnt_95% {st_qnt_95:.2f} range {st_qnt_95-st_qnt_05:.2f}')
print(f'quartile: lower {st_loqrt:.2f} upper {st_upqrt:.2f} range {st_upqrt-st_loqrt:.2f}')

print('histogram:')
print(' - bin counts ', h_count)
print(' - bin ranges ', h_range)

#plotting histograms with the same settings as above
plt.hist(data, bins=bin_nr, range=bin_range)

# plot the stats, uncomment if desired

# plt.axvline(st_mean, linestyle='dashed')
# plt.axvline(st_min, linestyle='dashed')
# plt.axvline(st_max, linestyle='dashed')

# plt.axvline(st_median, color='tab:orange', linestyle='dotted')
# plt.axvline(st_qnt_05, color='tab:orange', linestyle='dotted')
# plt.axvline(st_qnt_95, color='tab:orange', linestyle='dotted')

# min_ylim, max_ylim = plt.ylim()
# x_offset = 1.01
# y_offset = max_ylim*0.9
# plt.text(st_mean*x_offset, y_offset, f'mean = {st_mean:.1f}')
# plt.text(st_min*x_offset, y_offset, f'min = {st_min:.1f}')
# plt.text(st_max*x_offset, y_offset, f'max = {st_max:.1f}')

# y_offset = max_ylim*0.7
# plt.text(st_median*x_offset, y_offset, f'median = {st_median:.1f}', color='tab:orange')
# plt.text(st_qnt_05*x_offset, y_offset, f'qnt_05% = {st_qnt_05:.1f}', color='tab:orange')
# plt.text(st_qnt_95*x_offset, y_offset, f'qnt_95% = {st_qnt_95:.1f}', color='tab:orange')

plt.title(f'histogram bin_nr = {bin_nr:d}')

plt.show()
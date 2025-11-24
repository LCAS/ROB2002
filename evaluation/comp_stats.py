import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

# key parameters
true_count = 20 # this is our reference, true count or M
N = 20          # the total number of repetitions N
N_short = 5     # the short subset 

data_A = np.round(np.random.normal(20, 5, N)) # a standard set with the mean close to the true value
data_B = np.round(np.random.normal(24, 5, N)) # a second set with the mean away from the true count value

data_A_short = data_A[0:N_short] # subset of data N = 5
data_B_short = data_B[0:N_short]

print('Basic stats and standard error:')
print(f'A, N={len(data_A_short):d}: ', data_A_short.astype(int))
print(f' mean: {np.mean(data_A_short):.2f}, std: {np.std(data_A_short):.2f}, SE: {stats.sem(data_A_short):.2f}')
print(f'A, N={len(data_A):d}: ', data_A.astype(int))
print(f' mean: {np.mean(data_A):.2f}, std: {np.std(data_A):.2f}, SE: {stats.sem(data_A):.2f}')

print('\nError metrics:')
error_A = data_A - true_count
error_B = data_B - true_count

error_A_short = error_A[0:N_short]
error_B_short = error_B[0:N_short]

print(f'A, N={len(data_A_short):d}, MAE: {np.mean(np.abs(error_A_short)):.2f}, RMSE: {np.sqrt(np.mean(error_A_short ** 2)):.2f}')
print(f'A, N={len(data_A):d}, MAE: {np.mean(np.abs(error_A)):.2f}, RMSE: {np.sqrt(np.mean(error_A ** 2)):.2f}')

print(f'B, N={len(data_A_short):d}, MAE: {np.mean(np.abs(error_B_short)):.2f}, RMSE: {np.sqrt(np.mean(error_B_short ** 2)):.2f}')
print(f'B, N={len(data_A):d}, MAE: {np.mean(np.abs(error_B)):.2f}, RMSE: {np.sqrt(np.mean(error_B ** 2)):.2f}')

print('\nOne-sample Student t-test:')
print(f'A, N={len(data_A_short):d}, t: {stats.ttest_1samp(data_A_short, true_count).statistic:.2f}, p: {stats.ttest_1samp(data_A_short, true_count).pvalue:.2f}')
print(f'A, N={len(data_A):d}, t: {stats.ttest_1samp(data_A, true_count).statistic:.2f}, p: {stats.ttest_1samp(data_A, true_count).pvalue:.2f}')

print(f'B, N={len(data_B_short):d}, t: {stats.ttest_1samp(data_B_short, true_count).statistic:.2f}, p: {stats.ttest_1samp(data_B_short, true_count).pvalue:.2f}')
print(f'B, N={len(data_B):d}, t: {stats.ttest_1samp(data_B, true_count).statistic:.2f}, p: {stats.ttest_1samp(data_B, true_count).pvalue:.2f}')

print('\nTwo-sample Student t-test:')
print(f'AB, N={len(data_A_short):d}, t: {stats.ttest_ind(data_A_short, data_B_short).statistic:.2f}, p: {stats.ttest_ind(data_A_short, data_B_short).pvalue:.2f}')
print(f'AB, N={len(data_A):d}, t: {stats.ttest_ind(data_A, data_B).statistic:.2f}, p: {stats.ttest_ind(data_A, data_B).pvalue:.2f}')

# additional visualisations
# SE with increasing N
# sd = []
# se = []
# for i in range(3, len(data)+1):
#     sd.append(np.std(data[0:i]))
#     se.append(stats.sem(data[0:i]))

# print(sd, se)

# plt.plot(range(3, len(data)+1), sd, label='std')
# plt.plot(range(3, len(data)+1), se, label='SE')
# plt.xlim(3, 20)
# plt.xlabel('N')
# plt.legend()
# plt.show()
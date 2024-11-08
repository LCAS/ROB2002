import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

true_count = 20
data = np.round(np.random.normal(24, 5, 20))
data_B = np.array([27, 16, 20, 20, 18, 20, 27, 15, 28, 15, 20, 19, 14, 10, 13, 28, 24, 26, 23, 9])
data_B = data
data = np.array([10, 20, 17, 15, 24, 17, 25, 17, 20, 14, 21, 22, 26, 16, 19, 23, 20, 27, 20, 23])
data_short = data[0:5]

print(f'N = {len(data_short):2d}: ', data_short.astype(int))
print(f' mean: {np.mean(data_short):.2f}, std: {np.std(data_short):.2f}, SE: {stats.sem(data_short):.2f}')
print(f'N = {len(data):2d}: ', data.astype(int))
print(f' mean: {np.mean(data):.2f}, std: {np.std(data):.2f}, SE: {stats.sem(data):.2f}')

error = data - true_count
error_B = data_B - true_count

error_short = error[0:5]
error_short_B = error_B[0:5]

print(f'N=5 A, t: {stats.ttest_1samp(data_short, true_count).statistic:.2f}, p: {stats.ttest_1samp(data_short, true_count).pvalue:.2f}')
print(f'N=20 A, t: {stats.ttest_1samp(data, true_count).statistic:.2f}, p: {stats.ttest_1samp(data, true_count).pvalue:.2f}')

print(f'N=5 B, t: {stats.ttest_1samp(data_B[0:5], true_count).statistic:.2f}, p: {stats.ttest_1samp(data_B[0:5], true_count).pvalue:.2f}')
print(f'N=20 B, t: {stats.ttest_1samp(data_B, true_count).statistic:.2f}, p: {stats.ttest_1samp(data_B, true_count).pvalue:.2f}')

print(f'N=5 A, MAE: {np.mean(np.abs(error_short)):.2f}, RMSE: {np.sqrt(np.mean(error_short ** 2)):.2f}')
print(f'N=5 B, MAE: {np.mean(np.abs(error_short_B)):.2f}, RMSE: {np.sqrt(np.mean(error_short_B ** 2)):.2f}')

print(f'N=20 A, MAE: {np.mean(np.abs(error)):.2f}, RMSE: {np.sqrt(np.mean(error ** 2)):.2f}')
print(f'N=20 B, MAE: {np.mean(np.abs(error_B)):.2f}, RMSE: {np.sqrt(np.mean(error_B ** 2)):.2f}')

# SE using numpy only
# print('SE full', np.std(data)/np.sqrt(len(data)))
# print('SE short', np.std(data_short)/np.sqrt(len(data_short)))

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
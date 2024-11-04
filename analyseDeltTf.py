import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('gripper_pose.csv')

df['time'] = df['time'].astype(np.int64)

df['delta_time'] = df['time'].diff()

df['delta_time'].fillna(0, inplace=True)

df['delta_time'] = df['delta_time'] /1000000.0
plt.figure(figsize=(20, 6))

plt.plot(df['time'], df['delta_time'], marker='o', linestyle='-', color='b', label='Delta Time entre les poses')

plt.xlabel('Nombre de poses')
#plt.ylabel('Delta Time (nanosecondes)')
plt.ylabel('Delta Time (ms)')

plt.title('Delta de temps entre les poses successives du gripper')

plt.legend()

plt.grid(True)

plt.savefig('graphs/delta_time_poses.png', format='png')

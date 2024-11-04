import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('gripper_pose.csv')

def calculate_distance(row1, row2):
    return np.sqrt((row2['tx'] - row1['tx'])**2 + 
                   (row2['ty'] - row1['ty'])**2 + 
                   (row2['tz'] - row1['tz'])**2)

df['time'] = df['time'].astype(np.int64)
df['time_diff'] = df['time'].diff() / 1e9  

df['distance'] = df.apply(lambda row: 0 if row.name == 0 else calculate_distance(df.iloc[row.name - 1], row), axis=1)

df['velocity'] = (df['distance'] / df['time_diff'])*100
df['velocity'].fillna(0, inplace=True) 

df['velocity_diff'] = df['velocity'].diff()
df['acceleration'] = df['velocity_diff'] / df['time_diff']
df['acceleration'].fillna(0, inplace=True)


plt.figure(figsize=(20, 6))
plt.plot(df['time'] / 1e9, df['velocity'], marker='o', linestyle='-', color='b', label='Vitesse du bras')
plt.xlabel('Temps (secondes)')
plt.ylabel('Vitesse (cm/s)')
plt.title('Vitesse du bras du robot en fonction du temps')
plt.grid(True)
plt.legend()

plt.savefig('graphs/vitesse_bras.png')

plt.figure(figsize=(20, 6))
plt.plot(df['time'] / 1e9, df['acceleration'], marker='o', linestyle='-', color='r', label='Accélération du bras')
plt.xlabel('Temps (secondes)')
plt.ylabel('Accélération (cm/s²)')
plt.title('Accélération du bras du robot en fonction du temps')
plt.grid(True)
plt.legend()
plt.savefig('graphs/acceleration_bras_cm_s2.png')
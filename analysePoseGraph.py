import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import imageio
import os

df = pd.read_csv('merged_poses.csv')

num_poses = len(df)

images = []

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def draw_camera_reference(ax):
    ax.quiver(0, 0, 0, 1, 0, 0, color='Red', linestyle='dotted', label='Caméra X')  # X 
    ax.quiver(0, 0, 0, 0, 1, 0, color='Green', linestyle='dotted', label='Caméra Y')  # Y 
    ax.quiver(0, 0, 0, 0, 0, 1, color='Blue', linestyle='dotted', label='Caméra Z')  # Z

for i in range(num_poses):
    ax.cla()  
    
    draw_camera_reference(ax)

    gripper_translation = np.array([df['gripper_tx'].iloc[i], df['gripper_ty'].iloc[i], df['gripper_tz'].iloc[i]])
    tag_translation = np.array([df['tag_tx'].iloc[i], df['tag_ty'].iloc[i], df['tag_tz'].iloc[i]])

    gripper_rotation = np.array([df['gripper_rx'].iloc[i], df['gripper_ry'].iloc[i], df['gripper_rz'].iloc[i]])
    tag_rotation = np.array([df['tag_rx'].iloc[i], df['tag_ry'].iloc[i], df['tag_rz'].iloc[i]])

    gripper_r = R.from_euler('xyz', gripper_rotation).as_matrix()
    tag_r = R.from_euler('xyz', tag_rotation).as_matrix()

    ax.quiver(gripper_translation[0], gripper_translation[1], gripper_translation[2], 
              gripper_r[0, 0], gripper_r[1, 0], gripper_r[2, 0], color='r', label='Gripper X')
    ax.quiver(gripper_translation[0], gripper_translation[1], gripper_translation[2], 
              gripper_r[0, 1], gripper_r[1, 1], gripper_r[2, 1], color='g', label='Gripper Y')
    ax.quiver(gripper_translation[0], gripper_translation[1], gripper_translation[2], 
              gripper_r[0, 2], gripper_r[1, 2], gripper_r[2, 2], color='b', label='Gripper Z')

    ax.quiver(tag_translation[0], tag_translation[1], tag_translation[2], 
              tag_r[0, 0], tag_r[1, 0], tag_r[2, 0], color='r', linestyle='--', label='Tag X' )
    ax.quiver(tag_translation[0], tag_translation[1], tag_translation[2], 
              tag_r[0, 1], tag_r[1, 1], tag_r[2, 1], color='g', linestyle='--', label='Tag Y')
    ax.quiver(tag_translation[0], tag_translation[1], tag_translation[2], 
              tag_r[0, 2], tag_r[1, 2], tag_r[2, 2], color='b', linestyle='--', label='Tag Z')

    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([0, 1.5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Comparaison des axes Gripper (vérité terrain) et Tag (détection)\nPose {i+1}/{num_poses}')
    ax.legend()

    plt.savefig(f'tempImage/temp_{i}.png')
    images.append(imageio.imread(f'tempImage/temp_{i}.png'))
    print("Enregistrement pose i")

imageio.mimsave('gripper_tag_comparison.gif', images, duration=0.5)

for i in range(num_poses):
    os.remove(f'tempImage/temp_{i}.png')

print("Le GIF a été généré et sauvegardé sous le nom 'gripper_tag_comparison.gif'.")

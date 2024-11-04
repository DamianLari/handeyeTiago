import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# Charger les données du CSV
df = pd.read_csv('merged_poses_new.csv')

# Sélectionner une paire de rotations gripper/tag à comparer
gripper_rotation = np.array([df['gripper_rx'].iloc[0], df['gripper_ry'].iloc[0], df['gripper_rz'].iloc[0]])
tag_rotation = np.array([df['tag_rx'].iloc[0], df['tag_ry'].iloc[0], df['tag_rz'].iloc[0]])

# Créer les rotations sous forme de quaternions
gripper_r = R.from_euler('ZYX', gripper_rotation)
tag_r = R.from_euler('ZYX', tag_rotation)

# Calculer l'erreur angulaire totale (erreur globale)
relative_rotation = gripper_r.inv() * tag_r
error_angle_total = np.linalg.norm(relative_rotation.as_rotvec())

print(f"Erreur angulaire totale calculée : {error_angle_total:.4f} radians")

# Calculer l'erreur angulaire pour chaque axe
error_angle_x = abs(gripper_rotation[0] - tag_rotation[0])
error_angle_y = abs(gripper_rotation[1] - tag_rotation[1])
error_angle_z = abs(gripper_rotation[2] - tag_rotation[2])

# Afficher les erreurs par axe
print(f"Erreur angulaire sur l'axe X : {error_angle_x:.4f} radians")
print(f"Erreur angulaire sur l'axe Y : {error_angle_y:.4f} radians")
print(f"Erreur angulaire sur l'axe Z : {error_angle_z:.4f} radians")

# Visualiser les axes de rotation (gripper et tag)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Tracé des axes pour le gripper (vérité terrain)
gripper_axes = gripper_r.as_matrix()
ax.quiver(0, 0, 0, gripper_axes[0, 0], gripper_axes[1, 0], gripper_axes[2, 0], color='r', label='Gripper X')
ax.quiver(0, 0, 0, gripper_axes[0, 1], gripper_axes[1, 1], gripper_axes[2, 1], color='g', label='Gripper Y')
ax.quiver(0, 0, 0, gripper_axes[0, 2], gripper_axes[1, 2], gripper_axes[2, 2], color='b', label='Gripper Z')

# Tracé des axes pour le tag (ArUco détecté)
tag_axes = tag_r.as_matrix()
ax.quiver(0, 0, 0, tag_axes[0, 0], tag_axes[1, 0], tag_axes[2, 0], color='r', linestyle='--', label='Tag X')
ax.quiver(0, 0, 0, tag_axes[0, 1], tag_axes[1, 1], tag_axes[2, 1], color='g', linestyle='--', label='Tag Y')
ax.quiver(0, 0, 0, tag_axes[0, 2], tag_axes[1, 2], tag_axes[2, 2], color='b', linestyle='--', label='Tag Z')

# Ajuster la visualisation
ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Comparaison des axes Gripper (vérité terrain) et Tag (détection)')
ax.legend()

plt.show()

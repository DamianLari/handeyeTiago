import pandas as pd
import numpy as np
import matplotlib
import os
from scipy.stats import linregress
matplotlib.use('Agg')  # Utilise un backend sans interface graphique
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# Créer le dossier pour les graphiques s'il n'existe pas
if not os.path.exists('graphs'):
    os.makedirs('graphs')

# Fonction pour calculer la distance euclidienne (erreur 3D)
def calc_3d_error(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)

# Fonction pour calculer l'erreur angulaire entre deux rotations en radians
def calc_angular_error(rot1, rot2):
    r1 = R.from_euler('ZYX', rot1)
    r2 = R.from_euler('ZYX', rot2)
    relative_rot = r1.inv() * r2
    angle = np.linalg.norm(relative_rot.as_rotvec())  # Retourne l'angle en radians
    #return np.degrees(angle)  
    return angle

def createTwoGraphs():
    fig_translation = plt.figure()
    ax_translation = fig_translation.add_subplot(111, projection='3d')

    # Tracé des translations (positions) des tags ArUco et de la pince
    ax_translation.plot(df['tag_tx'], df['tag_ty'], df['tag_tz'], label='Tag ArUco')
    ax_translation.plot(df['gripper_tx'], df['gripper_ty'], df['gripper_tz'], label='Pince')

    # Étiquettes et légende
    ax_translation.set_xlabel('X')
    ax_translation.set_ylabel('Y')
    ax_translation.set_zlabel('Z')
    ax_translation.set_title('Positions 3D (Translation)')
    ax_translation.legend()

    # Création de la figure pour les rotations
    fig_rotation = plt.figure()
    ax_rotation = fig_rotation.add_subplot(111, projection='3d')

    # Tracé des rotations des tags ArUco et de la pince
    ax_rotation.plot(df['tag_rx'], df['tag_ry'], df['tag_rz'], label='Tag ArUco')
    ax_rotation.plot(df['gripper_rx'], df['gripper_ry'], df['gripper_rz'], label='Pince')

    # Étiquettes et légende
    ax_rotation.set_xlabel('Rotation X')
    ax_rotation.set_ylabel('Rotation Y')
    ax_rotation.set_zlabel('Rotation Z')
    ax_rotation.set_title('Rotations 3D')
    ax_rotation.legend()

    # Sauvegarder les graphiques
    fig_translation.savefig('graphs/translation_graph.png')
    fig_rotation.savefig('graphs/rotation_graph.png')
    print("Les graphiques 3D ont été sauvegardés sous forme d'images.")

def plotTransla():
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 10))  # 3 rows for X, Y, Z translations
    translation_colors = ['red', 'green', 'blue']

    df.plot(y=['tag_tx', 'gripper_tx'], ax=axes[0], marker='o', color=translation_colors, title='Translation X (en mètres)')
    df.plot(y=['tag_ty', 'gripper_ty'], ax=axes[1], marker='o', color=translation_colors, title='Translation Y (en mètres)')
    df.plot(y=['tag_tz', 'gripper_tz'], ax=axes[2], marker='o', color=translation_colors, title='Translation Z (en mètres)')

    plt.tight_layout()  # Ajuster la mise en page pour éviter la superposition
    plt.savefig("graphs/translation.png")
    print("Les graphiques de translation ont été sauvegardés.")

def plotRota():
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 10))  # 3 rows for X, Y, Z rotations
    rotation_colors = ['red', 'green', 'blue']

    df.plot(y=['tag_rx', 'gripper_rx'], ax=axes[0], marker='o', color=rotation_colors, title='Rotation X (en radians)')
    df.plot(y=['tag_ry', 'gripper_ry'], ax=axes[1], marker='o', color=rotation_colors, title='Rotation Y (en radians)')
    df.plot(y=['tag_rz', 'gripper_rz'], ax=axes[2], marker='o', color=rotation_colors, title='Rotation Z (en radians)')

    plt.tight_layout()  # Ajuster la mise en page pour éviter la superposition
    plt.savefig("graphs/rotation.png")
    print("Les graphiques de rotation ont été sauvegardés.")

def calc_distance_from_camera(gripper_tx, gripper_ty, gripper_tz):
    return np.sqrt(gripper_tx**2 + gripper_ty**2 + gripper_tz**2)

def ErreurDistance():
    df['error_3d'] = df.apply(lambda row: calc_3d_error(np.array([row['gripper_tx'], row['gripper_ty'], row['gripper_tz']]), 
                                                        np.array([row['tag_tx'], row['tag_ty'], row['tag_tz']])), axis=1)

    plt.scatter(df['distance_from_camera'], df['error_3d'], color='blue', label='Erreur 3D')

    # Ajuster une régression linéaire pour modéliser la relation entre la distance et l'erreur
    slope, intercept, r_value, p_value, std_err = linregress(df['distance_from_camera'], df['error_3d'])

    plt.plot(df['distance_from_camera'], intercept + slope * df['distance_from_camera'], color='red', label=f'Regression line (slope={slope:.4f})')

    plt.xlabel('Distance du Gripper à la Caméra (m)')
    plt.ylabel('Erreur de Détection 3D (m)')
    plt.title('Relation entre la distance du Gripper et l\'Erreur de Détection de l\'ArUco')
    plt.legend()

    print(f"Coefficient de corrélation (R^2) : {r_value**2:.4f}")
    print(f"Pente de la régression (taux d'augmentation de l'erreur par mètre de distance) : {slope:.4f} m d'erreur par mètre de distance")

def plotErreurDistance():
    df['distance_from_camera_tag'] = df.apply(lambda row: calc_distance_from_camera(row['tag_tx'], row['tag_ty'], row['tag_tz']), axis=1)

    df_sorted = df.sort_values(by='distance_from_camera')

    plt.figure(figsize=(10, 6))
    plt.plot(df_sorted['distance_from_camera'], df_sorted['distance_from_camera_tag'], label='Distance du Tag (ArUco détecté)', color='red', marker='o')
    plt.plot(df_sorted['distance_from_camera'], df_sorted['distance_from_camera'], label='Distance du Tag (vérité terrain)', color='blue', linestyle='dotted', marker='x')

    plt.xlabel('Distance réele entre le tag et la caméra (m)')
    plt.ylabel('Distance estimée entre le tag et la caméra (m)')
    plt.title('Comparaison des distances entre l\'aruco (vérité terrain) et le tag (détecté) à mesure que l\'aruco s\'éloigne de la caméra')
    plt.legend()

    plt.tight_layout()
    plt.savefig("graphs/Erreur3dDistance.png")
    print("Le graphique comparant les distances a été sauvegardé sous forme d'image.")

df = pd.read_csv('merged_poses.csv')

df['distance_from_camera'] = df.apply(lambda row: calc_distance_from_camera(row['gripper_tx'], row['gripper_ty'], row['gripper_tz']), axis=1)

# Calcul des erreurs 3D et angulaires et de leurs moyennes
df['error_3d'] = df.apply(lambda row: calc_3d_error(np.array([row['gripper_tx'], row['gripper_ty'], row['gripper_tz']]), 
                                                    np.array([row['tag_tx'], row['tag_ty'], row['tag_tz']])), axis=1)
df['error_angular'] = df.apply(lambda row: calc_angular_error(np.array([row['gripper_rx'], row['gripper_ry'], row['gripper_rz']]), 
                                                              np.array([row['tag_rx'], row['tag_ry'], row['tag_rz']])), axis=1)

df['mean_3d_error'] = df['error_3d'].expanding().mean()
df['mean_angular_error'] = df['error_angular'].expanding().mean()

df.to_csv('pose_errors.csv', index=False)

print(f"Erreur moyenne 3D finale : {df['mean_3d_error'].iloc[-1]:.4f}")
print(f"Erreur moyenne angulaire finale : {df['mean_angular_error'].iloc[-1]:.4f}")

# Générer les graphiques et les analyses
createTwoGraphs()
plotTransla()
plotRota()
ErreurDistance()
plotErreurDistance()
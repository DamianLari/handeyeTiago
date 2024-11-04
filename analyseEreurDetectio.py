import pandas as pd
import numpy as np
import matplotlib
import os
from scipy.stats import linregress
matplotlib.use('Agg')  # Utilise un backend sans interface graphique
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import math

if not os.path.exists('graphs'):
    os.makedirs('graphs')

def calc_3d_error(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)

def calc_angular_error(rot1, rot2):
    r1 = R.from_euler('ZYX', rot1)
    r2 = R.from_euler('ZYX', rot2)
    relative_rot = r1.inv() * r2
    angle = np.linalg.norm(relative_rot.as_rotvec())  
    #return np.degrees(angle)  
    return angle

def createTwoGraphs():
    fig_translation = plt.figure()
    ax_translation = fig_translation.add_subplot(111, projection='3d')

    ax_translation.plot(df['tag_tx'], df['tag_ty'], df['tag_tz'], label='Tag ArUco')
    ax_translation.plot(df['gripper_tx'], df['gripper_ty'], df['gripper_tz'], label='Pince')

    ax_translation.set_xlabel('X')
    ax_translation.set_ylabel('Y')
    ax_translation.set_zlabel('Z')
    ax_translation.set_title('Positions 3D (Translation)')
    ax_translation.legend()

    fig_rotation = plt.figure()
    ax_rotation = fig_rotation.add_subplot(111, projection='3d')

    ax_rotation.plot(df['tag_rx'], df['tag_ry'], df['tag_rz'], label='Tag ArUco')
    ax_rotation.plot(df['gripper_rx'], df['gripper_ry'], df['gripper_rz'], label='Pince')

    ax_rotation.set_xlabel('Rotation X')
    ax_rotation.set_ylabel('Rotation Y')
    ax_rotation.set_zlabel('Rotation Z')
    ax_rotation.set_title('Rotations 3D')
    ax_rotation.legend()

    fig_translation.savefig('graphs/translation_graph.png')
    fig_rotation.savefig('graphs/rotation_graph.png')
    print("Les graphiques 3D ont été sauvegardés sous forme d'images.")

def plotTransla():
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 10))  
    translation_colors = ['red', 'green', 'blue']

    df.plot(x='time',y=['tag_tx', 'gripper_tx'], ax=axes[0], marker='o', color=translation_colors, title='Translation X (en mètres)')
    df.plot(x='time',y=['tag_ty', 'gripper_ty'], ax=axes[1], marker='o', color=translation_colors, title='Translation Y (en mètres)')
    df.plot(x='time',y=['tag_tz', 'gripper_tz'], ax=axes[2], marker='o', color=translation_colors, title='Translation Z (en mètres)')

    plt.tight_layout()  
    plt.savefig("graphs/translation.png")
    print("Les graphiques de translation ont été sauvegardés.")

def plotRota():
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 10))  
    rotation_colors = ['red', 'green', 'blue']

    df.plot(x='time',y=['tag_rx', 'gripper_rx'], ax=axes[0], marker='o', color=rotation_colors, title='Rotation X (en radians)')
    df.plot(x='time',y=['tag_ry', 'gripper_ry'], ax=axes[1], marker='o', color=rotation_colors, title='Rotation Y (en radians)')
    df.plot(x='time',y=['tag_rz', 'gripper_rz'], ax=axes[2], marker='o', color=rotation_colors, title='Rotation Z (en radians)')

    plt.tight_layout()  
    plt.savefig("graphs/rotation.png")
    print("Les graphiques de rotation ont été sauvegardés.")

def barTransla():
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 10))  
    translation_colors = ['red', 'green']  # Couleurs pour le tag et le gripper
    bar_width = 0.3  # Réduction de la largeur des barres pour éviter le chevauchement
    index = np.arange(len(df['distance_from_camera']))  # Position des barres sur l'axe X

    # Graphique de la translation X
    axes[0].bar(index, df['tag_tx'], bar_width, color=translation_colors[0], label='Tag ArUco TX')
    axes[0].bar(index + bar_width, df['gripper_tx'], bar_width, color=translation_colors[1], label='Gripper TX')
    axes[0].set_xticks(index + bar_width / 2)
    axes[0].set_xticklabels(df['distance_from_camera'])
    axes[0].set_title('Translation X (en mètres) en fonction de la distance')
    axes[0].legend()

    # Graphique de la translation Y
    axes[1].bar(index, df['tag_ty'], bar_width, color=translation_colors[0], label='Tag ArUco TY')
    axes[1].bar(index + bar_width, df['gripper_ty'], bar_width, color=translation_colors[1], label='Gripper TY')
    axes[1].set_xticks(index + bar_width / 2)
    axes[1].set_xticklabels(df['distance_from_camera'])
    axes[1].set_title('Translation Y (en mètres) en fonction de la distance')
    axes[1].legend()

    # Graphique de la translation Z
    axes[2].bar(index, df['tag_tz'], bar_width, color=translation_colors[0], label='Tag ArUco TZ')
    axes[2].bar(index + bar_width, df['gripper_tz'], bar_width, color=translation_colors[1], label='Gripper TZ')
    axes[2].set_xticks(index + bar_width / 2)
    axes[2].set_xticklabels(df['distance_from_camera'])
    axes[2].set_title('Translation Z (en mètres) en fonction de la distance')
    axes[2].legend()

    # Ajustement de la disposition pour éviter que les barres ne se chevauchent
    plt.tight_layout()  
    plt.savefig("graphs/translation_bar_distance_cote_a_cote.png")
    print("Les graphiques de translation en barres côte à côte ont été sauvegardés.")

def barRotaTagOnly():
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 10))

    # Tracer la rotation en fonction de la position X du tag
    df.plot(x='gripper_tx', y='tag_rx', kind='bar', ax=axes[0], color='red', width=0.4, legend=False)
    axes[0].set_title('Rotation X du Tag ArUco (en radians) en fonction de la position X')
    
    # Tracer la rotation en fonction de la position Y du tag
    df.plot(x='gripper_ty', y='tag_ry', kind='bar', ax=axes[1], color='green', width=0.4, legend=False)
    axes[1].set_title('Rotation Y du Tag ArUco (en radians) en fonction de la position Y')
    
    # Tracer la rotation en fonction de la position Z du tag
    df.plot(x='gripper_tz', y='tag_rz', kind='bar', ax=axes[2], color='blue', width=0.4, legend=False)
    axes[2].set_title('Rotation Z du Tag ArUco (en radians) en fonction de la position Z')

    for ax in axes:
        ax.set_xlabel('Position du Tag ArUco (m)')
        ax.set_ylabel('Rotation (radians)')
    
    plt.tight_layout()
    plt.savefig("graphs/rotation_tag_by_axes.png")
    print("Les graphiques de rotation du Tag ArUco en fonction de ses positions X, Y et Z ont été sauvegardés.")


def barRotaBoth():
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 10))  
    rotation_colors = ['red', 'green']  # Couleurs pour le tag et le gripper
    bar_width = 0.3  # Réduction de la largeur des barres pour éviter le chevauchement
    index = np.arange(len(df['distance_from_camera']))  # Position des barres sur l'axe X

    # Graphique de la rotation X
    axes[0].bar(index, df['tag_rx'], bar_width, color=rotation_colors[0], label='Tag ArUco RX')
    axes[0].bar(index + bar_width, df['gripper_rx'], bar_width, color=rotation_colors[1], label='Gripper RX')
    axes[0].set_xticks(index + bar_width / 2)
    axes[0].set_xticklabels(df['distance_from_camera'])
    axes[0].set_title('Rotation X (en radians) en fonction de la distance')
    axes[0].legend()

    # Graphique de la rotation Y
    axes[1].bar(index, df['tag_ry'], bar_width, color=rotation_colors[0], label='Tag ArUco RY')
    axes[1].bar(index + bar_width, df['gripper_ry'], bar_width, color=rotation_colors[1], label='Gripper RY')
    axes[1].set_xticks(index + bar_width / 2)
    axes[1].set_xticklabels(df['distance_from_camera'])
    axes[1].set_title('Rotation Y (en radians) en fonction de la distance')
    axes[1].legend()

    # Graphique de la rotation Z
    axes[2].bar(index, df['tag_rz'], bar_width, color=rotation_colors[0], label='Tag ArUco RZ')
    axes[2].bar(index + bar_width, df['gripper_rz'], bar_width, color=rotation_colors[1], label='Gripper RZ')
    axes[2].set_xticks(index + bar_width / 2)
    axes[2].set_xticklabels(df['distance_from_camera'])
    axes[2].set_title('Rotation Z (en radians) en fonction de la distance')
    axes[2].legend()

    # Ajustement de la disposition pour éviter que les barres ne se chevauchent
    plt.tight_layout()  
    plt.savefig("graphs/rotation_bar_distance_cote_a_cote.png")
    print("Les graphiques de rotation en barres côte à côte ont été sauvegardés.")


def calc_distance_from_camera(gripper_tx, gripper_ty, gripper_tz):
    return np.sqrt(gripper_tx**2 + gripper_ty**2 + gripper_tz**2)

def ErreurDistance():
    df['error_3d'] = df.apply(lambda row: calc_3d_error(np.array([row['gripper_tx'], row['gripper_ty'], row['gripper_tz']]), 
                                                        np.array([row['tag_tx'], row['tag_ty'], row['tag_tz']])), axis=1)

    plt.scatter(df['distance_from_camera'], df['error_3d'], color='blue', label='Erreur 3D')

    slope, intercept, r_value, p_value, std_err = linregress(df['distance_from_camera'], df['error_3d'])

    plt.plot(df['distance_from_camera'], intercept + slope * df['distance_from_camera'], color='red', label=f'Regression line (slope={slope:.4f})')

    plt.xlabel('Distance du Gripper à la Caméra (m)')
    plt.ylabel('Erreur de Détection 3D (m)')
    plt.title('Relation entre la distance du Gripper et l\'Erreur de Détection de l\'ArUco')
    plt.legend()

    print(f"Coefficient de corrélation (R^2) : {r_value**2:.4f}")
    print(f"Pente de la régression (taux d'augmentation de l'erreur par mètre de distance) : {slope:.4f} m d'erreur par mètre de distance")

def Erreur6DOF(row):
    Hrot_Gripper = np.eye(3)
    Hrot_Tag = np.eye(3)

    Hrot_Gripper[0:3, 0:3] = R.from_euler("ZYX", [row['gripper_rz'], row['gripper_ry'], row['gripper_rx']], degrees=False).as_matrix()
    #H_Gripper[0, 3] = row['gripper_tx']
    #H_Gripper[1, 3] = row['gripper_ty']
    #H_Gripper[2, 3] = row['gripper_tz']

    Hrot_Tag[0:3, 0:3] = R.from_euler("ZYX", [row['tag_rz'], row['tag_ry'], row['tag_rx']], degrees=False).as_matrix()
    #H_Tag[0, 3] = row['tag_tx']
    #H_Tag[1, 3] = row['tag_ty']
    #H_Tag[2, 3] = row['tag_tz']
    
    Hroterreur = Hrot_Tag.dot(np.linalg.inv(Hrot_Gripper))
    

    Ex = row['tag_tx'] - row['gripper_tx']
    Ey = row['tag_tx'] - row['gripper_tx']
    Ez = row['tag_tx'] - row['gripper_tx']

    E_3D = math.sqrt(Ex**2 + Ey**2 + Ez**2)
    Erpy = np.flip(R.from_matrix(Hroterreur[0:3, 0:3]).as_euler("ZYX", degrees=True))
    ErotVec = R.from_matrix(Hroterreur[0:3, 0:3]).as_rotvec(degrees=True)
    angleSolide = np.linalg.norm(ErotVec)

    if ErotVec[2] < 0:  
        angleSolide = -angleSolide

    return E_3D, angleSolide, Erpy[0], Erpy[1], Erpy[2]

def plot_6DOF_errors():
    plt.figure(figsize=(20, 8))

    plt.subplot(3, 1, 1)
    plt.plot(df['time'], df['E_3D'], label='Erreur 3D (m)', marker='o')
    plt.xlabel('Temps')
    plt.ylabel('Erreur 3D (m)')
    plt.title('Erreur de position entre gripper et tag')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(df['time'], df['Erx'], label='Erreur en Roll (deg)', marker='o')
    plt.plot(df['time'], df['Ery'], label='Erreur en Pitch (deg)', marker='o')
    plt.plot(df['time'], df['Erz'], label='Erreur en Yaw (deg)', marker='o')
    plt.xlabel('Temps')
    plt.ylabel('Erreurs d\'angles (deg)')
    plt.title('Erreurs d\'angles entre gripper et tag')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(df['time'], df['angleSolide'], label='Erreur angle solide (deg)', marker='o')
    plt.xlabel('Temps')
    plt.ylabel('Angle solide (deg)')
    plt.title('Erreur angle solide entre gripper et tag')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig('graphs/Erreur_6DOF.png')
    print("Les erreurs de 6DOF ont été calculées et les graphiques ont été sauvegardés.")


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


df = pd.read_csv('merged_poses_new.csv')

df[['E_3D', 'angleSolide', 'Erx', 'Ery', 'Erz']] = df.apply(lambda row: pd.Series(Erreur6DOF(row)), axis=1)
df.to_csv('pose_errors_6DOF.csv', index=False)

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
barTransla()
plotRota()
barRotaTagOnly()
barRotaBoth()
#ErreurDistance()
plotErreurDistance()
plot_6DOF_errors()
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import csv

use_degrees = True
unit_label = "degrés" if use_degrees else "radians"

def erreur_6dof(target, row, next_row):
    # Calculer la matrice de rotation pour les positions n et n+1
    Hrot_Tag_n = R.from_euler("ZYX", [row[f'{target}_rz'], row[f'{target}_ry'], row[f'{target}_rx']], degrees=use_degrees).as_matrix()
    Hrot_Tag_n_plus_1 = R.from_euler("ZYX", [next_row[f'{target}_rz'], next_row[f'{target}_ry'], next_row[f'{target}_rx']], degrees=use_degrees).as_matrix()

    # Calcul de l'erreur de rotation
    Hroterreur = Hrot_Tag_n_plus_1 @ np.linalg.inv(Hrot_Tag_n)

    # Calcul de la translation en 3D
    E_3D = np.linalg.norm([next_row[f'{target}_tx'] - row[f'{target}_tx'],
                           next_row[f'{target}_ty'] - row[f'{target}_ty'],
                           next_row[f'{target}_tz'] - row[f'{target}_tz']])

    # Calcul de l'angle solide
    ErotVec = R.from_matrix(Hroterreur).as_rotvec()
    angle_solide = np.linalg.norm(ErotVec)
    if ErotVec[2] < 0:
        angle_solide = -angle_solide
    if use_degrees:
        angle_solide = np.degrees(angle_solide)
    
    return E_3D, angle_solide

def process_csv(input_file, output_file):
    # Lecture du fichier CSV
    df = pd.read_csv(input_file)
    result_rows = []

    # Calcul des erreurs pour chaque ligne
    for i in range(len(df) - 1):
        row = df.iloc[i]
        next_row = df.iloc[i + 1]
        E_3D, angle_solide = erreur_6dof('tag', row, next_row)
        result_rows.append({
            'image_n': row['image'],
            'image_n_plus_1': next_row['image'],
            'erreur_3D': E_3D,
            'erreur_solide': angle_solide
        })

    # Sauvegarde du fichier CSV des résultats
    result_df = pd.DataFrame(result_rows)
    result_df.to_csv(output_file, index=False)
    print(f"Les résultats ont été sauvegardés dans le fichier {output_file}")

def compute_values(target, df):
    translations = []
    angles_solides = []
    steps = list(range(len(df) - 1))

    # Calcul des translations et angles solides pour chaque étape
    for i in steps:
        row = df.iloc[i]
        next_row = df.iloc[i + 1]
        E_3D, angle_solide = erreur_6dof(target, row, next_row)
        translations.append(E_3D)
        angles_solides.append(angle_solide)

    return steps, translations, angles_solides

def save_solid_angles_to_csv(steps, translations_tag, angles_solides_tag, translations_gt, angles_solides_gt, output_file='solid_angles.csv'):
    # Création du DataFrame et sauvegarde en CSV
    result_df = pd.DataFrame({
        'Étape': steps,
        'Translation (Tag)': translations_tag,
        'Angle solide (Tag)': angles_solides_tag,
        'Translation (GT)': translations_gt,
        'Angle solide (GT)': angles_solides_gt
    })
    result_df.to_csv(output_file, index=False)
    print(f"Les valeurs d'angles solides et de translations ont été sauvegardées dans {output_file}")

def generate_combined_plot(df, steps, angles_solides_tag, angles_solides_gt):
    fig, axs = plt.subplots(3, 2, figsize=(16, 18))
    
    # Graphiques de rotation pour les axes X, Y, Z
    axes = ['x', 'y', 'z']
    for i, axis in enumerate(axes):
        axs[i, 0].plot(df.index, df[f'tag_r{axis}'], color='red', label=f'tag_r{axis}')
        axs[i, 0].plot(df.index, df[f'gripper_r{axis}'], color='green', label=f'gripper_r{axis}')
        axs[i, 0].set_title(f"Rotation {axis.upper()} (en {unit_label})")
        axs[i, 0].set_xlabel("Time")
        axs[i, 0].set_ylabel(f"Rotation ({unit_label})")
        axs[i, 0].legend()

    # Graphiques des angles solides
    axs[0, 1].plot(df.index, df['angleSolide'], color='blue', label='tag/gripper angle solide')
    axs[0, 1].set_title("Angle solide (tag et gripper)")
    axs[0, 1].set_xlabel("Time")
    axs[0, 1].set_ylabel(f"Angle solide ({unit_label})")
    axs[0, 1].legend()

    axs[1, 1].plot(steps, angles_solides_tag, 'o-', color='purple', label='Angle solide entre n et n+1 (Tag)')
    axs[1, 1].set_title("Variation de l'Angle Solide entre Étapes Successives (Tag)")
    axs[1, 1].set_xlabel("Images")
    axs[1, 1].set_ylabel(f"Angle solide ({unit_label})")
    axs[1, 1].legend()

    axs[2, 1].plot(steps, angles_solides_gt, 'o-', color='orange', label='Angle solide entre n et n+1 (Gripper)')
    axs[2, 1].set_title("Variation de l'Angle Solide entre Étapes Successives (Gripper)")
    axs[2, 1].set_xlabel("Images")
    axs[2, 1].set_ylabel(f"Angle solide ({unit_label})")
    axs[2, 1].legend()

    plt.tight_layout()
    plt.savefig('combined_graphs.png')
    print("Les graphiques combinés ont été sauvegardés sous 'combined_graphs.png'.")

input_file = 'merged_poses.csv'
df = pd.read_csv(input_file)

# Calcul des erreurs et angles solides
df[['E_3D', 'angleSolide', 'Erx', 'Ery', 'Erz']] = df.apply(lambda row: pd.Series(erreur_6dof('tag', row, row)), axis=1)

if use_degrees:
    df[['tag_rx', 'tag_ry', 'tag_rz', 'gripper_rx', 'gripper_ry', 'gripper_rz', 'angleSolide']] = np.degrees(df[['tag_rx', 'tag_ry', 'tag_rz', 'gripper_rx', 'gripper_ry', 'gripper_rz', 'angleSolide']])

steps, translations_tag, angles_solides_tag, translations_gt, angles_solides_gt = compute_values('tag', df), compute_values('gripper', df)

save_solid_angles_to_csv(steps, translations_tag, angles_solides_tag, translations_gt, angles_solides_gt)
generate_combined_plot(df, steps, angles_solides_tag, angles_solides_gt)

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import csv


use_degrees = True
unit_label = "degrés" if use_degrees else "radians"

def Erreur6DOF(target,row, next_row):
    Hrot_Tag_n = np.eye(3)
    Hrot_Tag_n_plus_1 = np.eye(3)

    Hrot_Tag_n[0:3, 0:3] = R.from_euler("ZYX", [row[f'{target}_rz'], row[f'{target}_ry'], row[f'{target}_rx']], degrees=use_degrees).as_matrix()
    Hrot_Tag_n_plus_1[0:3, 0:3] = R.from_euler("ZYX", [next_row[f'{target}_rz'], next_row[f'{target}_ry'], next_row[f'{target}_rx']], degrees=use_degrees).as_matrix()

    Hroterreur = Hrot_Tag_n_plus_1.dot(np.linalg.inv(Hrot_Tag_n))

    Ex = next_row[f'{target}_tx'] - row[f'{target}_tx']
    Ey = next_row[f'{target}_ty'] - row[f'{target}_ty']
    Ez = next_row[f'{target}_tz'] - row[f'{target}_tz']
    E_3D = math.sqrt(Ex**2 + Ey**2 + Ez**2)

    ErotVec = R.from_matrix(Hroterreur[0:3, 0:3]).as_rotvec()
    angleSolide = np.linalg.norm(ErotVec)
    if ErotVec[2] < 0:
        angleSolide = -angleSolide
    if use_degrees:
        angleSolide = np.degrees(angleSolide)
    
    return E_3D, angleSolide

def process_csv(input_file, output_file):
    result_rows = []

    with open(input_file, mode='r') as infile:
        reader = csv.DictReader(infile)
        rows = list(reader)
        
        image_column_name = list(rows[0].keys())[0]  

        for i in range(len(rows) - 1):
            image_n = rows[i][image_column_name]
            image_n_plus_1 = rows[i + 1][image_column_name]
            
            row = {k: float(v) for k, v in list(rows[i].items())[1:]}
            next_row = {k: float(v) for k, v in list(rows[i + 1].items())[1:]}
            
            E_3D, angleSolide = Erreur6DOF(row, next_row)
            result_rows.append({
                'image_n': image_n,
                'image_n_plus_1': image_n_plus_1,
                'erreur_3D': E_3D,
                'erreur_solide': angleSolide
            })

    with open(output_file, mode='w', newline='') as outfile:
        fieldnames = ['image_n', 'image_n_plus_1', 'erreur_3D', 'erreur_solide']
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(result_rows)

    print(f"Les résultats ont été sauvegardés dans le fichier {output_file}")

def compute_values(target,rows):
    translations = []
    angles_solides = []
    steps = []
    for i in range(len(rows) - 1):
            row = {k: float(v) for k, v in list(rows[i].items())[1:]}
            next_row = {k: float(v) for k, v in list(rows[i + 1].items())[1:]}
            E_3D, angleSolide = Erreur6DOF(target,row, next_row)
            
            translations.append(E_3D)
            if use_degrees:
                angleSolide = np.degrees(angleSolide)
            angles_solides.append(angleSolide)
            steps.append(i)
    return steps, translations, angles_solides

def process(input_file):
    with open(input_file, mode='r') as infile:
        reader = csv.DictReader(infile)
        rows = list(reader)
        steps_tag, translations_tag,angles_solides_tag = compute_values('tag',rows)        
        steps_gt, translations_gt,angles_solides_gt = compute_values('gripper',rows)

    return steps_gt,translations_tag, angles_solides_tag , translations_gt,angles_solides_gt

def save_solid_angles_to_csv(steps, translations_tag, angles_solides_tag, translations_gt, angles_solides_gt, output_file='solid_angles.csv'):
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Étape', 'Translation (Tag)', 'Angle solide (Tag)', 'Translation (GT)', 'Angle solide (GT)'])
        
        for step, trans_tag, angle_tag, trans_gt, angle_gt in zip(steps, translations_tag, angles_solides_tag, translations_gt, angles_solides_gt):
            writer.writerow([step, trans_tag, angle_tag, trans_gt, angle_gt])
    
    print(f"Les valeurs d'angles solides et de translations ont été sauvegardées dans {output_file}")

def plotRota():
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 10))  
    rotation_colors = ['red', 'green', 'blue']
    
    df.plot(x='time',y=['tag_rx', 'gripper_rx'], ax=axes[0], marker='o', color=rotation_colors, title='Rotation X (en radians)')
    df.plot(x='time',y=['tag_ry', 'gripper_ry'], ax=axes[1], marker='o', color=rotation_colors, title='Rotation Y (en radians)')
    df.plot(x='time',y=['tag_rz', 'gripper_rz'], ax=axes[2], marker='o', color=rotation_colors, title='Rotation Z (en radians)')

    plt.tight_layout()  
    plt.savefig("graphs/rotation.png")
    print("Les graphiques de rotation ont été sauvegardés.")

def DOFErreur(row):
    Hrot_Gripper = np.eye(3)
    Hrot_Tag = np.eye(3)

    Hrot_Gripper[0:3, 0:3] = R.from_euler("ZYX", [row['gripper_rz'], row['gripper_ry'], row['gripper_rx']], degrees=use_degrees).as_matrix()
    #H_Gripper[0, 3] = row['gripper_tx']
    #H_Gripper[1, 3] = row['gripper_ty']
    #H_Gripper[2, 3] = row['gripper_tz']

    Hrot_Tag[0:3, 0:3] = R.from_euler("ZYX", [row['tag_rz'], row['tag_ry'], row['tag_rx']], degrees=use_degrees).as_matrix()
    #H_Tag[0, 3] = row['tag_tx']
    #H_Tag[1, 3] = row['tag_ty']
    #H_Tag[2, 3] = row['tag_tz']
    
    Hroterreur = Hrot_Tag.dot(np.linalg.inv(Hrot_Gripper))
    

    Ex = row['tag_tx'] - row['gripper_tx']
    Ey = row['tag_tx'] - row['gripper_tx']
    Ez = row['tag_tx'] - row['gripper_tx']

    E_3D = math.sqrt(Ex**2 + Ey**2 + Ez**2)
    Erpy = np.flip(R.from_matrix(Hroterreur[0:3, 0:3]).as_euler("ZYX", degrees=use_degrees))
    ErotVec = R.from_matrix(Hroterreur[0:3, 0:3]).as_rotvec(degrees=use_degrees)
    angleSolide = np.linalg.norm(ErotVec)

    if ErotVec[2] < 0:  
        angleSolide = -angleSolide

    return E_3D, angleSolide, Erpy[0], Erpy[1], Erpy[2]

def calc_distance_from_camera(gripper_tx, gripper_ty, gripper_tz):
    return np.sqrt(gripper_tx**2 + gripper_ty**2 + gripper_tz**2)

def calc_3d_error(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)

def calc_angular_error(rot1, rot2):
    r1 = R.from_euler('ZYX', rot1)
    r2 = R.from_euler('ZYX', rot2)
    relative_rot = r1.inv() * r2
    angle = np.linalg.norm(relative_rot.as_rotvec())  
    
    return angle

def generate_combined_plot(df, steps, angles_solides_tag, angles_solides_gt):
    # Création de la figure avec 3 lignes et 3 colonnes pour les neuf graphiques
    fig, axs = plt.subplots(3, 3, figsize=(18, 18))
    
    # 1. Graphique de rotation pour l'axe X (tag et gripper)
    axs[0, 0].plot(df.index, df['tag_rx'], color='red', label='tag_rx')
    axs[0, 0].plot(df.index, df['gripper_rx'], color='green', label='gripper_rx')
    axs[0, 0].set_title(f"Rotation X (en {unit_label})")
    axs[0, 0].set_xlabel("Images")
    axs[0, 0].set_ylabel(f"Rotation ({unit_label})")
    axs[0, 0].legend()

    # 2. Graphique de différence de rotation pour l'axe X
    axs[0, 1].plot(df.index, df['diff_rx'], color='red', label='Différence RX (tag - gripper)')
    axs[0, 1].set_title(f"Différence de Rotation X (en {unit_label})")
    axs[0, 1].set_xlabel("Images")
    axs[0, 1].set_ylabel(f"Différence Rotation ({unit_label})")
    axs[0, 1].axhline(0, color='black', linestyle='--')  # Ligne de référence à 0
    axs[0, 1].legend()

    # 3. Graphique de rotation pour l'axe Y (tag et gripper)
    axs[1, 0].plot(df.index, df['tag_ry'], color='red', label='tag_ry')
    axs[1, 0].plot(df.index, df['gripper_ry'], color='green', label='gripper_ry')
    axs[1, 0].set_title(f"Rotation Y (en {unit_label})")
    axs[1, 0].set_xlabel("Images")
    axs[1, 0].set_ylabel(f"Rotation ({unit_label})")
    axs[1, 0].legend()

    # 4. Graphique de différence de rotation pour l'axe Y
    axs[1, 1].plot(df.index, df['diff_ry'], color='green', label='Différence RY (tag - gripper)')
    axs[1, 1].set_title(f"Différence de Rotation Y (en {unit_label})")
    axs[1, 1].set_xlabel("Images")
    axs[1, 1].set_ylabel(f"Différence Rotation ({unit_label})")
    axs[1, 1].axhline(0, color='black', linestyle='--')  # Ligne de référence à 0
    axs[1, 1].legend()

    # 5. Graphique de rotation pour l'axe Z (tag et gripper)
    axs[2, 0].plot(df.index, df['tag_rz'], color='red', label='tag_rz')
    axs[2, 0].plot(df.index, df['gripper_rz'], color='green', label='gripper_rz')
    axs[2, 0].set_title(f"Rotation Z (en {unit_label})")
    axs[2, 0].set_xlabel("Images")
    axs[2, 0].set_ylabel(f"Rotation ({unit_label})")
    axs[2, 0].legend()

    # 6. Graphique de différence de rotation pour l'axe Z
    axs[2, 1].plot(df.index, df['diff_rz'], color='blue', label='Différence RZ (tag - gripper)')
    axs[2, 1].set_title(f"Différence de Rotation Z (en {unit_label})")
    axs[2, 1].set_xlabel("Images")
    axs[2, 1].set_ylabel(f"Différence Rotation ({unit_label})")
    axs[2, 1].axhline(0, color='black', linestyle='--')  # Ligne de référence à 0
    axs[2, 1].legend()

    # 7-9. Graphiques des angles solides (tag et gripper)
    axs[0, 2].plot(df.index, df['angleSolide'], color='blue', label='tag/gripper angle solide')
    axs[0, 2].set_title("Angle Solide entre Tag et Gripper")
    axs[0, 2].set_xlabel("Images")
    axs[0, 2].set_ylabel(f"Angle solide ({unit_label})")
    axs[0, 2].legend()

    axs[1, 2].plot(steps, angles_solides_gt, 'o-', color='orange', label='Angle solide (Gripper/Verité Terrain)')
    axs[1, 2].set_title("Variation de l'Angle Solide entre Étapes Successives (Gripper)")
    axs[1, 2].set_xlabel("Images")
    axs[1, 2].set_ylabel(f"Angle solide ({unit_label})")
    axs[1, 2].legend()

    axs[2, 2].plot(steps, angles_solides_tag, 'o-', color='purple', label='Angle solide (Tag)')
    axs[2, 2].set_title("Variation de l'Angle Solide entre Étapes Successives (Tag)")
    axs[2, 2].set_xlabel("Images")
    axs[2, 2].set_ylabel(f"Angle solide ({unit_label})")
    axs[2, 2].legend()

    # Ajustement de l'espacement et sauvegarde de l'image
    plt.tight_layout()
    plt.savefig('combined_graphs.png')
    print("Les graphiques combinés ont été sauvegardés sous 'combined_graphs.png'.")


def save_combined_errors_to_csv(df, output_file='combined_errors.csv'):
    result_rows = []
    for index, row in df.iterrows():
        _,angleSolide, tag_erreur_rx, tag_erreur_ry, tag_erreur_rz = DOFErreur(row)
        result_row = {
            'image': row['image_file'],  # Assurez-vous que la colonne 'image' existe dans df
            'gripper_rx': row['gripper_rx'],
            'gripper_ry': row['gripper_ry'],
            'gripper_rz': row['gripper_rz'],
            'tag_erreur_rx': tag_erreur_rx,
            'tag_erreur_ry': tag_erreur_ry,
            'tag_erreur_rz': tag_erreur_rz,
            'erreur_angle_solide': angleSolide
        }
        result_rows.append(result_row)

    with open(output_file, mode='w', newline='') as file:
        fieldnames = ['image', 'gripper_rx', 'gripper_ry', 'gripper_rz', 'tag_erreur_rx', 'tag_erreur_ry', 'tag_erreur_rz', 'erreur_angle_solide']
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(result_rows)
    
    print(f"Les erreurs combinées ont été sauvegardées dans {output_file}")

input_file = 'merged_poses.csv'
df = pd.read_csv(input_file)

df[['E_3D', 'angleSolide', 'Erx', 'Ery', 'Erz']] = df.apply(lambda row: pd.Series(DOFErreur(row)), axis=1)

if use_degrees:
    df[['tag_rx', 'tag_ry', 'tag_rz', 'gripper_rx', 'gripper_ry', 'gripper_rz','angleSolide']] = np.degrees(df[['tag_rx', 'tag_ry', 'tag_rz', 'gripper_rx', 'gripper_ry', 'gripper_rz','angleSolide']])

df['diff_rx'] = df['tag_rx'] - df['gripper_rx']
df['diff_ry'] = df['tag_ry'] - df['gripper_ry']
df['diff_rz'] = df['tag_rz'] - df['gripper_rz']

steps,translations_tag, angles_solides_tag, translations_gt,angles_solides_gt = process(input_file)
save_solid_angles_to_csv(steps, translations_tag, angles_solides_tag, translations_gt,angles_solides_gt)

generate_combined_plot(df, steps, angles_solides_tag, angles_solides_gt)

save_combined_errors_to_csv(df)
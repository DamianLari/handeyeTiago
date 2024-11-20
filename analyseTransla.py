import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import csv

use_degrees = True
unit_label = "degrés" if use_degrees else "radians"

def Erreur6DOF(target, row, next_row):
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

def compute_values(target, rows):
    translations = []
    angles_solides = []
    steps = []
    for i in range(len(rows) - 1):
        row = {k: float(v) for k, v in list(rows[i].items())[1:]}
        next_row = {k: float(v) for k, v in list(rows[i + 1].items())[1:]}
        E_3D, angleSolide = Erreur6DOF(target, row, next_row)
        
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
        steps_tag, translations_tag, angles_solides_tag = compute_values('tag', rows)        
        steps_gt, translations_gt, angles_solides_gt = compute_values('gripper', rows)

    return steps_gt, translations_tag, angles_solides_tag, translations_gt, angles_solides_gt

def save_solid_angles_to_csv(steps, translations_tag, angles_solides_tag, translations_gt, angles_solides_gt, output_file='solid_angles.csv'):
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Étape', 'Translation (Tag)', 'Angle solide (Tag)', 'Translation (GT)', 'Angle solide (GT)'])
        
        for step, trans_tag, angle_tag, trans_gt, angle_gt in zip(steps, translations_tag, angles_solides_tag, translations_gt, angles_solides_gt):
            writer.writerow([step, trans_tag, angle_tag, trans_gt, angle_gt])
    
    print(f"Les valeurs d'angles solides et de translations ont été sauvegardées dans {output_file}")

def DOFErreur(row):
    Hrot_Gripper = np.eye(3)
    Hrot_Tag = np.eye(3)

    Hrot_Gripper[0:3, 0:3] = R.from_euler("ZYX", [row['gripper_rz'], row['gripper_ry'], row['gripper_rx']], degrees=use_degrees).as_matrix()
    Hrot_Tag[0:3, 0:3] = R.from_euler("ZYX", [row['tag_rz'], row['tag_ry'], row['tag_rx']], degrees=use_degrees).as_matrix()
    
    Hroterreur = Hrot_Tag.dot(np.linalg.inv(Hrot_Gripper))

    Ex = row['tag_tx'] - row['gripper_tx']
    Ey = row['tag_ty'] - row['gripper_ty']
    Ez = row['tag_tz'] - row['gripper_tz']

    E_3D = math.sqrt(Ex**2 + Ey**2 + Ez**2)
    Erpy = np.flip(R.from_matrix(Hroterreur[0:3, 0:3]).as_euler("ZYX", degrees=use_degrees))
    ErotVec = R.from_matrix(Hroterreur[0:3, 0:3]).as_rotvec(degrees=use_degrees)
    angleSolide = np.linalg.norm(ErotVec)

    if ErotVec[2] < 0:  
        angleSolide = -angleSolide

    return E_3D, angleSolide, Erpy[0], Erpy[1], Erpy[2]

def generate_translation_plots(df, steps, translations_tag, translations_gt):
    # Création de la figure avec 3 lignes et 3 colonnes pour les neuf graphiques de translations
    fig, axs = plt.subplots(3, 3, figsize=(18, 18))
    
    # 1. Graphique de translation pour l'axe X (tag et gripper)
    axs[0, 0].plot(df.index, df['tag_tx'], color='red', label='tag_tx')
    axs[0, 0].plot(df.index, df['gripper_tx'], color='green', label='gripper_tx')
    axs[0, 0].set_title("Translation X")
    axs[0, 0].set_xlabel("Images")
    axs[0, 0].set_ylabel("Translation (m)")
    axs[0, 0].legend()

    # 2. Graphique de différence de translation pour l'axe X
    axs[0, 1].plot(df.index, df['diff_tx'], color='blue', label='Différence TX (tag - gripper)')
    axs[0, 1].set_title("Différence de Translation X")
    axs[0, 1].set_xlabel("Images")
    axs[0, 1].set_ylabel("Différence Translation (m)")
    axs[0, 1].axhline(0, color='black', linestyle='--')  # Ligne de référence à 0
    axs[0, 1].legend()

    # 3. Graphique de translation pour l'axe Y (tag et gripper)
    axs[1, 0].plot(df.index, df['tag_ty'], color='red', label='tag_ty')
    axs[1, 0].plot(df.index, df['gripper_ty'], color='green', label='gripper_ty')
    axs[1, 0].set_title("Translation Y")
    axs[1, 0].set_xlabel("Images")
    axs[1, 0].set_ylabel("Translation (m)")
    axs[1, 0].legend()

    # 4. Graphique de différence de translation pour l'axe Y
    axs[1, 1].plot(df.index, df['diff_ty'], color='blue', label='Différence TY (tag - gripper)')
    axs[1, 1].set_title("Différence de Translation Y")
    axs[1, 1].set_xlabel("Images")
    axs[1, 1].set_ylabel("Différence Translation (m)")
    axs[1, 1].axhline(0, color='black', linestyle='--')  # Ligne de référence à 0
    axs[1, 1].legend()

    # 5. Graphique de translation pour l'axe Z (tag et gripper)
    axs[2, 0].plot(df.index, df['tag_tz'], color='red', label='tag_tz')
    axs[2, 0].plot(df.index, df['gripper_tz'], color='green', label='gripper_tz')
    axs[2, 0].set_title("Translation Z")
    axs[2, 0].set_xlabel("Images")
    axs[2, 0].set_ylabel("Translation (m)")
    axs[2, 0].legend()

    # 6. Graphique de différence de translation pour l'axe Z
    axs[2, 1].plot(df.index, df['diff_tz'], color='blue', label='Différence TZ (tag - gripper)')
    axs[2, 1].set_title("Différence de Translation Z")
    axs[2, 1].set_xlabel("Images")
    axs[2, 1].set_ylabel("Différence Translation (m)")
    axs[2, 1].axhline(0, color='black', linestyle='--')  # Ligne de référence à 0
    axs[2, 1].legend()

    # 7. Graphique de l'erreur 3D pour le tag
    axs[0, 2].plot(steps, translations_tag, 'o-', color='purple', label='Erreur 3D (Tag)')
    axs[0, 2].set_title("Erreur 3D entre Étapes Successives (Tag)")
    axs[0, 2].set_xlabel("Images")
    axs[0, 2].set_ylabel("Erreur 3D (m)")
    axs[0, 2].legend()

    # 8. Graphique de l'erreur 3D pour le gripper
    axs[1, 2].plot(steps, translations_gt, 'o-', color='orange', label='Erreur 3D (Gripper)')
    axs[1, 2].set_title("Erreur 3D entre Étapes Successives (Gripper)")
    axs[1, 2].set_xlabel("Images")
    axs[1, 2].set_ylabel("Erreur 3D (m)")
    axs[1, 2].legend()

    # 9. Graphique comparatif de l'erreur 3D entre le tag et le gripper
    axs[2, 2].plot(steps, translations_tag, 'o-', color='purple', label='Erreur 3D (Tag)')
    axs[2, 2].plot(steps, translations_gt, 'o-', color='orange', label='Erreur 3D (Gripper)')
    axs[2, 2].set_title("Comparaison des Erreurs 3D entre Tag et Gripper")
    axs[2, 2].set_xlabel("Images")
    axs[2, 2].set_ylabel("Erreur 3D (m)")
    axs[2, 2].legend()

    # Ajustement de l'espacement et sauvegarde de l'image
    plt.tight_layout()
    plt.savefig('translation_graphs.png')
    print("Les graphiques de translations ont été sauvegardés sous 'translation_graphs.png'.")

input_file = 'merged_poses.csv'
df = pd.read_csv(input_file)

# Calcul des erreurs DOF et ajout des colonnes au DataFrame
df[['E_3D', 'angleSolide', 'Erx', 'Ery', 'Erz']] = df.apply(lambda row: pd.Series(DOFErreur(row)), axis=1)

# Convertir les angles en degrés si nécessaire
if use_degrees:
    df[['tag_rx', 'tag_ry', 'tag_rz', 'gripper_rx', 'gripper_ry', 'gripper_rz', 'angleSolide']] = np.degrees(df[['tag_rx', 'tag_ry', 'tag_rz', 'gripper_rx', 'gripper_ry', 'gripper_rz', 'angleSolide']])

# Calcul des différences pour chaque translation

df['diff_tx'] = df['tag_tx'] - df['gripper_tx']
df['diff_ty'] = df['tag_ty'] - df['gripper_ty']
df['diff_tz'] = df['tag_tz'] - df['gripper_tz']

# Processus pour obtenir les valeurs nécessaires pour les graphiques
steps, translations_tag, angles_solides_tag, translations_gt, angles_solides_gt = process(input_file)

# Sauvegarde des angles solides dans un fichier CSV
save_solid_angles_to_csv(steps, translations_tag, angles_solides_tag, translations_gt, angles_solides_gt)

# Génération des graphiques pour les translations
generate_translation_plots(df, steps, translations_tag, translations_gt)

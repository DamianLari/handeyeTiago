import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import matplotlib.pyplot as plt

def Erreur6DOF(row, next_row):
    Hrot_Tag_n = np.eye(3)
    Hrot_Tag_n_plus_1 = np.eye(3)

    Hrot_Tag_n[0:3, 0:3] = R.from_euler("ZYX", [row['tag_rz'], row['tag_ry'], row['tag_rx']], degrees=False).as_matrix()
    Hrot_Tag_n_plus_1[0:3, 0:3] = R.from_euler("ZYX", [next_row['tag_rz'], next_row['tag_ry'], next_row['tag_rx']], degrees=False).as_matrix()

    Hroterreur = Hrot_Tag_n_plus_1.dot(np.linalg.inv(Hrot_Tag_n))

    Ex = next_row['tag_tx'] - row['tag_tx']
    Ey = next_row['tag_ty'] - row['tag_ty']
    Ez = next_row['tag_tz'] - row['tag_tz']
    E_3D = math.sqrt(Ex**2 + Ey**2 + Ez**2)

    ErotVec = R.from_matrix(Hroterreur[0:3, 0:3]).as_rotvec()
    angleSolide = np.linalg.norm(ErotVec)
    if ErotVec[2] < 0:
        angleSolide = -angleSolide

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

def process(input_file):
    translations = []
    angles_solides = []
    steps = []

    with open(input_file, mode='r') as infile:
        reader = csv.DictReader(infile)
        rows = list(reader)
        
        for i in range(len(rows) - 1):
            row = {k: float(v) for k, v in list(rows[i].items())[1:]}
            next_row = {k: float(v) for k, v in list(rows[i + 1].items())[1:]}
            E_3D, angleSolide = Erreur6DOF(row, next_row)
            
            translations.append(E_3D)
            angles_solides.append(angleSolide)
            steps.append(i)

    return steps, translations, angles_solides

def plot_results(steps, translations, angles_solides):
    plt.figure(figsize=(12, 6))

    plt.subplot(1, 2, 1)
    plt.plot(steps, translations, marker='o', color='b', label="Erreur de translation (3D)")
    plt.xlabel("Étape")
    plt.ylabel("Erreur de translation (m)")
    plt.title("Erreur de Translation 3D entre n et n+1")
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(steps, angles_solides, marker='o', color='r', label="Angle solide")
    plt.xlabel("Étape")
    plt.ylabel("Angle solide (radians)")
    plt.title("Angle Solide entre n et n+1")
    plt.legend()

    plt.tight_layout()
    plt.savefig('angles_solid.png')

input_file = 'merged_poses.csv'
steps, translations, angles_solides = process(input_file)
plot_results(steps, translations, angles_solides)

input_file = 'merged_poses.csv'
output_file = 'erreurs_translation_rotation.csv'
process_csv(input_file, output_file)

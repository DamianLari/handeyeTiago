import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
import csv

# Fonction pour calculer l'erreur 3D entre deux positions successives
def Erreur3D(target, row, next_row):
    # Calcul des différences de translation entre deux étapes
    Ex = next_row[f'{target}_tx'] - row[f'{target}_tx']
    Ey = next_row[f'{target}_ty'] - row[f'{target}_ty']
    Ez = next_row[f'{target}_tz'] - row[f'{target}_tz']
    E_3D = math.sqrt(Ex**2 + Ey**2 + Ez**2)
    return E_3D

# Fonction pour calculer les valeurs de translation 3D entre étapes successives
def compute_values(target, rows):
    translations_x = []
    translations_y = []
    translations_z = []
    translations_3D = []
    steps = []
    for i in range(len(rows) - 1):
        row = {k: float(v) for k, v in list(rows[i].items())[1:]}
        next_row = {k: float(v) for k, v in list(rows[i + 1].items())[1:]}
        
        # Calcul des translations pour chaque axe
        tx = next_row[f'{target}_tx'] - row[f'{target}_tx']
        ty = next_row[f'{target}_ty'] - row[f'{target}_ty']
        tz = next_row[f'{target}_tz'] - row[f'{target}_tz']
        E_3D = math.sqrt(tx**2 + ty**2 + tz**2)
        
        translations_x.append(tx)
        translations_y.append(ty)
        translations_z.append(tz)
        translations_3D.append(E_3D)
        steps.append(i)
    
    return steps, translations_x, translations_y, translations_z, translations_3D

# Fonction principale de traitement des données
def process(input_file):
    with open(input_file, mode='r') as infile:
        reader = csv.DictReader(infile)
        rows = list(reader)
        
        # Calculs pour le tag
        steps_tag, translations_x_tag, translations_y_tag, translations_z_tag, translations_3D_tag = compute_values('tag', rows)
        
        # Calculs pour la GT
        steps_gt, translations_x_gt, translations_y_gt, translations_z_gt, translations_3D_gt = compute_values('gripper', rows)
    
    return steps_tag, translations_x_tag, translations_y_tag, translations_z_tag, translations_3D_tag, translations_x_gt, translations_y_gt, translations_z_gt, translations_3D_gt

# Fonction pour générer les graphiques de translation
def generate_combined_translation_plots(df, steps, translations_x_tag, translations_y_tag, translations_z_tag, translations_3D_tag,
                                        translations_x_gt, translations_y_gt, translations_z_gt, translations_3D_gt):
    # Création de la figure avec 3 lignes et 2 colonnes pour les six graphiques
    fig, axs = plt.subplots(3, 2, figsize=(16, 18))
    
    # 1. Graphique pour les translations en X
    axs[0, 0].plot(steps, translations_x_tag, 'o-', color='blue', label='Translation X (Tag)')
    axs[0, 0].plot(steps, translations_x_gt, 'o-', color='orange', label='Translation X (GT)')
    axs[0, 0].set_title("Comparaison des Translations en X")
    axs[0, 0].set_xlabel("Étapes")
    axs[0, 0].set_ylabel("Translation (mètres)")
    axs[0, 0].legend()

    # 2. Graphique pour les translations en Y
    axs[1, 0].plot(steps, translations_y_tag, 'o-', color='blue', label='Translation Y (Tag)')
    axs[1, 0].plot(steps, translations_y_gt, 'o-', color='orange', label='Translation Y (GT)')
    axs[1, 0].set_title("Comparaison des Translations en Y")
    axs[1, 0].set_xlabel("Étapes")
    axs[1, 0].set_ylabel("Translation (mètres)")
    axs[1, 0].legend()

    # 3. Graphique pour les translations en Z
    axs[2, 0].plot(steps, translations_z_tag, 'o-', color='blue', label='Translation Z (Tag)')
    axs[2, 0].plot(steps, translations_z_gt, 'o-', color='orange', label='Translation Z (GT)')
    axs[2, 0].set_title("Comparaison des Translations en Z")
    axs[2, 0].set_xlabel("Étapes")
    axs[2, 0].set_ylabel("Translation (mètres)")
    axs[2, 0].legend()

    # 4. Graphique pour l'erreur 3D entre le Tag et la GT
    axs[0, 1].plot(steps, translations_3D_tag, 'o-', color='blue', label='Erreur 3D (Tag)')
    axs[0, 1].plot(steps, translations_3D_gt, 'o-', color='orange', label='Erreur 3D (GT)')
    axs[0, 1].set_title("Erreur 3D entre le Tag et la GT")
    axs[0, 1].set_xlabel("Étapes")
    axs[0, 1].set_ylabel("Erreur 3D (mètres)")
    axs[0, 1].legend()

    # 5. Graphique pour l'erreur 3D entre n et n+1 pour le Tag
    axs[1, 1].plot(steps, translations_3D_tag, 'o-', color='purple', label='Erreur 3D n et n+1 (Tag)')
    axs[1, 1].set_title("Erreur 3D entre Étapes Successives (Tag)")
    axs[1, 1].set_xlabel("Étapes")
    axs[1, 1].set_ylabel("Erreur 3D (mètres)")
    axs[1, 1].legend()

    # 6. Graphique pour l'erreur 3D entre n et n+1 pour la GT
    axs[2, 1].plot(steps, translations_3D_gt, 'o-', color='purple', label='Erreur 3D n et n+1 (GT)')
    axs[2, 1].set_title("Erreur 3D entre Étapes Successives (GT)")
    axs[2, 1].set_xlabel("Étapes")
    axs[2, 1].set_ylabel("Erreur 3D (mètres)")
    axs[2, 1].legend()

    # Ajustement de l'espacement et sauvegarde de l'image
    plt.tight_layout()
    plt.savefig('combined_translations_3D.png')
    plt.show()
    print("Les graphiques combinés ont été sauvegardés sous 'combined_translations_3D.png'.")

# Exécution de l'ensemble du processus
input_file = 'merged_poses.csv'
df = pd.read_csv(input_file)

# Calcul des étapes et des erreurs de translation pour le Tag et la GT
steps, translations_x_tag, translations_y_tag, translations_z_tag, translations_3D_tag, translations_x_gt, translations_y_gt, translations_z_gt, translations_3D_gt = process(input_file)

# Génération des graphiques combinés
generate_combined_translation_plots(df, steps, translations_x_tag, translations_y_tag, translations_z_tag, translations_3D_tag,
                                    translations_x_gt, translations_y_gt, translations_z_gt, translations_3D_gt)

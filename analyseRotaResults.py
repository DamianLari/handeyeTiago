import pandas as pd
import matplotlib.pyplot as plt

use_degrees = True  # ou False selon ce que vous voulez
unit_label = "degrés" if use_degrees else "radians"
# Charger les données
df = pd.read_csv('combined_errors.csv')

# Calcul des statistiques descriptives
stats = df[['tag_erreur_rx', 'tag_erreur_ry', 'tag_erreur_rz', 'erreur_angle_solide']].describe()

# Afficher les statistiques
print("Statistiques descriptives :\n", stats)

# Analyse de la corrélation entre les erreurs et les rotations du gripper
correlation_matrix = df[['gripper_rx', 'gripper_ry', 'gripper_rz', 'tag_erreur_rx', 'tag_erreur_ry', 'tag_erreur_rz']].corr()
print("\nMatrice de corrélation :\n", correlation_matrix)

# Graphiques des erreurs de rotation par axe
fig, ax = plt.subplots(3, 1, figsize=(10, 8))
axes = ['x', 'y', 'z']
for i, axis in enumerate(axes):
    ax[i].plot(df['image'], df[f'tag_erreur_r{axis}'], label=f'Erreur rotation {axis.upper()}')
    ax[i].set_xlabel('Image')
    ax[i].set_ylabel(f'Erreur rotation {axis.upper()} ({unit_label})')
    ax[i].legend()
plt.tight_layout()
plt.show()

# Graphique de l'erreur d'angle solide
plt.figure(figsize=(10, 4))
plt.plot(df['image'], df['erreur_angle_solide'], label='Erreur angle solide', color='purple')
plt.xlabel('Image')
plt.ylabel(f'Erreur angle solide ({unit_label})')
plt.legend()
plt.show()

import numpy as np

def calculate_3d_reprojection_error(point_A_in_B, point_A_in_C_direct, transformation_matrix_B_to_C):
    """
    Calcule l'erreur de reprojection 3D entre un point A dans le repère C
    obtenu directement et celui obtenu par transformation du repère B vers C.
    
    :param point_A_in_B: Coordonnées du point A dans le repère B (numpy array de taille 3 ou 4).
    :param point_A_in_C_direct: Coordonnées du point A dans le repère C obtenu directement (numpy array de taille 3).
    :param transformation_matrix_B_to_C: Matrice de transformation du repère B vers C (numpy array de taille 4x4).
    :return: Erreur de reprojection 3D.
    """

    # Convertir le point A dans le repère B en coordonnées homogènes (ajouter 1 en 4ème position)
    if len(point_A_in_B) == 3:
        point_A_in_B_homogeneous = np.append(point_A_in_B, 1)
    else:
        point_A_in_B_homogeneous = point_A_in_B

    # Appliquer la transformation du repère B au repère C
    point_A_in_C_transformed = transformation_matrix_B_to_C @ point_A_in_B_homogeneous
    
    # Passer en coordonnées cartésiennes (diviser par la dernière composante si nécessaire)
    point_A_in_C_transformed_cartesian = point_A_in_C_transformed[:3] / point_A_in_C_transformed[3]

    # Calculer l'erreur de reprojection (distance euclidienne)
    error = np.linalg.norm(point_A_in_C_direct - point_A_in_C_transformed_cartesian)

    return error

# Exemple d'utilisation
point_A_in_B = np.array([1, 2, 4])  # Coordonnées du point A dans le repère B
point_A_in_C_direct = np.array([4, 5, 6])  # Coordonnées directes du point A dans le repère C

# Matrice de transformation du repère B vers C (4x4)
transformation_matrix_B_to_C = np.array([[1, 0, 0, 3],
                                         [0, 1, 0, 3],
                                         [0, 0, 1, 3],
                                         [0, 0, 0, 1]])

# Calculer l'erreur de reprojection 3D
error = calculate_3d_reprojection_error(point_A_in_B, point_A_in_C_direct, transformation_matrix_B_to_C)

print(f"Erreur de reprojection 3D : {error}")

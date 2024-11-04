from calibExtra_provider import calculate_3d_reprojection_and_angular_error
import numpy as np
from scipy.spatial.transform import Rotation as R

if __name__ == "__main__":
    translation_A_in_C = np.array([1, 2, 2])  # A dans repère C
    rotation_A_to_C = np.array([1, 2, 4])  # A dans repère C (angles Euler)
    translation_A_in_B = np.array([4, 5, 6])  # A dans repère B
    rotation_A_to_B = np.array([1, 2, 4])  # A dans repère B (angles Euler)
    """
    translation_A_in_C = np.array([0.12526152036823612,0.17706772492082745,0.5552287812004333])
    rotation_A_to_C = np.array([-1.2384867815397844,-2.4644754789869188,-0.8010735258874131])
    translation_A_in_B = np.array([0.24633599115884908,-0.24743553437407267,0.7949294261056582])
    rotation_A_to_B = np.array([0.10746207219126269,1.303738272039416,-3.0653998480106055])
    """

    """

    rotation_C_to_B = np.array([-2.26255134, 0.65167776, 1.55132634])
    translation_C_to_B = np.array([ 0.28651529, -0.08386277,  0.7228922 ])
    """
    rotation_C_to_B = np.array([0,0,0])
    translation_C_to_B = np.array([ 0,1,0])
    # Calculer la matrice de rotation à partir des angles d'Euler
    rotation_matrix_C_to_B = R.from_euler('xyz', rotation_C_to_B).as_matrix()

    # Construire la matrice de transformation du repère C vers B
    transformation_matrix_C_to_B = np.eye(4)
    transformation_matrix_C_to_B[:3, :3] = rotation_matrix_C_to_B
    transformation_matrix_C_to_B[:3, 3] = translation_C_to_B
    
    # transformation repère C vers B
    """
    transformation_matrix_C_to_B = np.array([[1, 0, 0, 3],
                                            [0, 1, 0, 3],
                                            [0, 0, 1, 3],
                                            [0, 0, 0, 1]])
    """
  
    # calculer l'erreur 3D et l'erreur angulaire
    reprojection_error, angular_error_degrees = calculate_3d_reprojection_and_angular_error(
        translation_A_in_C, rotation_A_to_C, 
        translation_A_in_B, rotation_A_to_B,
        transformation_matrix_C_to_B)

    print(f"Erreur de reprojection 3D : {reprojection_error}")
    print(f"Erreur angulaire : {angular_error_degrees}")

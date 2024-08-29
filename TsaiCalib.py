import numpy as np
from scipy.spatial.transform import Rotation as R

def TsaiHandEyeCalibration(R_gripper_to_base, t_gripper_to_base, R_target_to_cam, t_target_to_cam):
    """
    Calcule la calibration "eye-in-hand" en utilisant l'algorithme de Tsai.
    
    Args:
        R_gripper_to_base: Liste des matrices de rotation de la pince à la base (3x3 numpy arrays).
        t_gripper_to_base: Liste des vecteurs de translation de la pince à la base (3x1 numpy arrays).
        R_target_to_cam: Liste des matrices de rotation de la cible à la caméra (3x3 numpy arrays).
        t_target_to_cam: Liste des vecteurs de translation de la cible à la caméra (3x1 numpy arrays).
        
    Returns:
        R_cam_to_base: Matrice de rotation de la caméra à la base (3x3 numpy array).
        t_cam_to_base: Vecteur de translation de la caméra à la base (3x1 numpy array).
    """
    A = []
    B = []

    for Rg, tg, Rc, tc in zip(R_gripper_to_base, t_gripper_to_base, R_target_to_cam, t_target_to_cam):
        A.append(Rg)
        B.append(Rc)

    A = np.array(A)
    B = np.array(B)

    # Résoudre pour la rotation en utilisant la méthode décrite par Tsai
    R_cam_to_base = solve_rotation(A, B)
    t_cam_to_base = solve_translation(A, B, R_cam_to_base, t_gripper_to_base, t_target_to_cam)

    return R_cam_to_base, t_cam_to_base

def solve_rotation(A, B):
    M = np.zeros((3, 3))
    for i in range(len(A)):
        M += np.dot(B[i].T, A[i])
    
    U, S, Vt = np.linalg.svd(M)
    R_cam_to_base = np.dot(U, Vt)
    return R_cam_to_base

def solve_translation(A, B, R_cam_to_base, t_gripper_to_base, t_target_to_cam):
    n = len(A)
    C = np.zeros((3 * n, 3))
    d = np.zeros(3 * n)

    for i in range(n):
        C[3 * i: 3 * (i + 1), :] = np.eye(3) - A[i]
        d[3 * i: 3 * (i + 1)] = t_target_to_cam[i].flatten() - A[i].dot(t_gripper_to_base[i].flatten())

    t_cam_to_base, _, _, _ = np.linalg.lstsq(C, d, rcond=None)
    return t_cam_to_base


# Exemple d'utilisation
if __name__ == "__main__":
    # Supposons que nous ayons les matrices de rotation et les vecteurs de translation sous forme de listes
    R_gripper_to_base = [
        np.eye(3),
        # Ajouter d'autres matrices de rotation ici
    ]
    t_gripper_to_base = [
        np.array([0, 0, 0]),
        # Ajouter d'autres vecteurs de translation ici
    ]
    R_target_to_cam = [
        np.eye(3),
        # Ajouter d'autres matrices de rotation ici
    ]
    t_target_to_cam = [
        np.array([0, 0, 0]),
        # Ajouter d'autres vecteurs de translation ici
    ]

    R_cam_to_base, t_cam_to_base = TsaiHandEyeCalibration(
        R_gripper_to_base, t_gripper_to_base, 
        R_target_to_cam, t_target_to_cam
    )

    print("Matrice de rotation de la caméra à la base:", R_cam_to_base)
    print("Vecteur de translation de la caméra à la base:", t_cam_to_base)

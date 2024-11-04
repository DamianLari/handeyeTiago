import numpy as np
import math3d as m3d

class TsaiHandEyeCalibration(object):
    def __init__(self, pose_pairs=None):
        self.pose_pairs = pose_pairs

    def _solve(self):
        pp = self.pose_pairs
        M = len(pp)
        lhs = []
        rhs = []
        for i in range(M):
            for j in range(i + 1, M):
                Hgij = pp[j][0].inverse() * pp[i][0]
                Pgij = 2 * Hgij.orient.quaternion.vector_part
                Hcij = pp[j][1].inverse() * pp[i][1]
                Pcij = 2 * Hcij.orient.quaternion.vector_part
                lhs.append(skew(Pgij.data + Pcij.data))
                rhs.append(Pcij.data - Pgij.data)
        lhs = np.array(lhs)
        lhs = lhs.reshape(lhs.shape[0] * 3, 3)
        rhs = np.array(rhs)
        rhs = rhs.reshape(rhs.shape[0] * 3)
        Pcg_, res, rank, sing = np.linalg.lstsq(lhs, rhs, rcond=None)
        Pcg = 2 * Pcg_ / np.sqrt(1 + np.dot(Pcg_.reshape(3), Pcg_.reshape(3)))
        Rcg = quat_to_rot(Pcg / 2)

        # Calculer la composante translationnelle
        lhs = []
        rhs = []
        for i in range(M):
            for j in range(i + 1, M):
                Hgij = pp[j][0].inverse() * pp[i][0]
                Hcij = pp[j][1].inverse() * pp[i][1]
                lhs.append(Hgij.data[:3, :3] - np.eye(3))
                rhs.append(np.dot(Rcg[:3, :3], Hcij.pos.data) - Hgij.pos.data)
        lhs = np.array(lhs)
        lhs = lhs.reshape(lhs.shape[0] * 3, 3)
        rhs = np.array(rhs)
        rhs = rhs.reshape(rhs.shape[0] * 3)
        Tcg, res, rank, sing = np.linalg.lstsq(lhs, rhs, rcond=None)
        Hcg = m3d.Transform(np.ravel(Rcg[:3, :3]), Tcg)
        self._sif = Hcg

    @property
    def sensor_in_flange(self):
        self._solve()
        return self._sif

def skew(v):
    if len(v) == 4:
        v = v[:3] / v[3]
    skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
    return skv - skv.T

def quat_to_rot(q):
    q = np.array(q).reshape(3, 1)
    p = np.dot(q.T, q).reshape(1)[0]
    if p > 1:
        print('Warning: quaternion greater than 1')
    w = np.sqrt(1 - p)
    R = np.eye(4)
    R[:3, :3] = 2 * q * q.T + 2 * w * skew(q) + np.eye(3) - 2 * np.diag([p, p, p])
    return R

def rot_to_quat(r):
    w4 = 2 * np.sqrt(1 - np.trace(r[:3, :3]))
    q = np.array([(r[2, 1] - r[1, 2]) / w4,
                  (r[0, 2] - r[2, 0]) / w4,
                  (r[1, 0] - r[0, 1]) / w4])
    return q

def create_homogeneous_matrix(R, t):
    """
    Crée une matrice de transformation homogène 4x4 à partir de la matrice de rotation et du vecteur de translation.
    """
    H = np.eye(4)
    H[:3, :3] = R  # Rotation 3x3
    H[:3, 3] = t.flatten()  # Translation 3x1
    return m3d.Transform(H)

if __name__ == '__main__':
    # Supposons que nous ayons des rotations et translations sous forme de listes
    R_gripper_to_base = [
        np.eye(3),
        np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])  # Exemple de rotation
    ]
    t_gripper_to_base = [
        np.array([0, 0, 0]),
        np.array([1, 0, 0])
    ]
    R_target_to_cam = [
        np.eye(3),
        np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])  # Exemple de rotation différente
    ]
    t_target_to_cam = [
        np.array([0, 0, 0]),
        np.array([0, 1, 0])
    ]

    # Créer des paires de transformations homogènes
    pose_pairs = []
    for Rg, tg, Rc, tc in zip(R_gripper_to_base, t_gripper_to_base, R_target_to_cam, t_target_to_cam):
        pose_pairs.append((create_homogeneous_matrix(Rg, tg), create_homogeneous_matrix(Rc, tc)))

    # Utiliser le calibrateur Tsai-Lenz
    tlc = TsaiHandEyeCalibration(pose_pairs)
    sif_tsai = tlc.sensor_in_flange
    print(sif_tsai)

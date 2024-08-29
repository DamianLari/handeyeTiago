import cv2
from cv2 import aruco
import numpy as np
import scipy.spatial.transform as R
import os
#import apriltag
import json

class TagPoseProvider:
    def set_calibration_params(self, mtx, dist):
        """
        Définit les paramètres de calibration de la caméra.
        
        Args:
            mtx (ndarray): Matrice de calibration de la caméra.
            dist (ndarray): Coefficients de distorsion de la caméra.
        """
        self.mtx = mtx
        self.dist = dist
    
    def get_calibration_params(self):
        """
        Retourne les paramètres de calibration de la caméra.
        
        Returns:
            tuple: Un tuple contenant la matrice de calibration et les coefficients de distorsion.
        """
        return self.mtx, self.dist

    def set_aruco_params(self,size,dict):
        """
        Définit les paramètres pour la détection des marqueurs ArUco.
        
        Args:
            dict (dictionary): Dictionnaire ArUco utilisé pour la détection des marqueurs.
            size (float): Taille des marqueurs ArUco.
        """
        self.aruco_dict = dict
        self.aruco_size = size

    
    def set_apriltag_params(self,tag_size,tag_dict):
        """
        Définit les paramètres pour la détection des marqueurs AprilTag.
        
        Args:
            tag_size (float): Taille des marqueurs AprilTag.
            tag_dict (string): Nom du dictionnaire AprilTag utilisé pour la détection.
        """
        self.tag_size=tag_size
        options = apriltag.DetectorOptions(families=tag_dict,
                                border=1,
                                nthreads=4,
                                quad_decimate=1.0,
                                quad_blur=0.0,
                                refine_edges=True,
                                refine_decode=False,
                                refine_pose=True,
                                debug=False,
                                quad_contours=True)
        self.detector = apriltag.Detector(options)
    


    def set_tag_config(self, tag_type,tag_size, tag_dict=None):
        self.tag_type = tag_type
        if tag_type == 'aruco':
            if tag_dict is not None:
                aruco_dict = getattr(aruco, tag_dict)
                self.set_aruco_params(aruco.Dictionary(aruco_dict,10), float(tag_size))
            else:
                print('Le nom du dictionnaire ArUco n\'est pas défini')
        elif tag_type =='apriltag':
            self.set_apriltag_params(float(tag_size),tag_dict)
        else:
            print('Erreur au moment de définir les paramètres du tag')


    def load_global_config(self,tag_list,camera_list):
        """
        Charge et initialise les configurations des tags et des caméras.

        Cette méthode crée des instances de TagConfig pour chaque élément de 'tag_list'
        et des instances de CameraConfig pour chaque élément de 'camera_list'.

        Args:
            tag_list (list): Une liste de dictionnaires où chaque dictionnaire contient les informations
                            de configuration d'un tag, incluant son type, sa taille, et son dictionnaire.
            camera_list (list): Une liste de dictionnaires où chaque dictionnaire contient les informations
                                de configuration d'une caméra, incluant son nom, le dossier d'images,
                                et le fichier de calibration.

        Returns:
            tuple: Deux listes, la première contenant les instances de TagConfig et la deuxième contenant
                les instances de CameraConfig.
        """
     
        tag_config = [TagConfig(tag['type'],tag['size'],tag['dict']) for tag in tag_list]
        
        #Créer un objet CameraConfig pour chaque caméra
        #Pour chaque caméra, on doit fournir :
        #                                    le nom de la caméra
        #                                    le chemin du dossier contenant les images (le chemin depuis la racine du projet ou depuis le dossier contenant le script python)
        #                                    la matrice de calibration et les coefficients de distorsion
        #self.camera_config = [CameraConfig(camera['name'],camera['images_folder'],*self.load_calib_data(self.tag_detection_provider.get_grandparent_file_path(camera['calibration_file'],'calib'))) for camera in camera_list]
        camera_config = [CameraConfig(camera['name'],camera['images_folder'],*self.load_calib_data(camera['calibration_file'])) for camera in camera_list]

        return tag_config,camera_config

    def load_calib_data(self,calibration_file):
        #================================
        # Charge les données de calibration depuis le fichier spécifié.
        # Args:
        #   calibration_file: Chemin vers le fichier de calibration.
        # Returns:
        #   matr: Matrice de calibration.
        #   disto: Coefficients de distorsion.
        #================================
        try:
            with open(calibration_file) as f:
                data = json.load(f)
            matr = data["mtx"]
            disto = data["dist"]
            return np.array(matr) , np.array(disto)
          
        except Exception as e:
            print("Erreur lors de la lecture du fichier de calibration :", e)
            return None, None
        
    def get_grandparent_file_path(self, file_name, subfolder=''):
        #================================
        # Retourne le chemin absolu du fichier spécifié, en partant du répertoire parent du répertoire parent du répertoire courant.
        # Args:
        #   file_name: Nom du fichier.
        #   subfolder: Nom du sous-dossier. Par défaut, il est vide.
        # Returns:
        #   Chemin absolu du fichier.
        #================================
        current_script_path = os.path.realpath(__file__)
        parent_directory_path = os.path.dirname(current_script_path)
        grandparent_directory_path = os.path.dirname(parent_directory_path)

        if subfolder:
            grandparent_directory_path = os.path.join(grandparent_directory_path, subfolder)

        return os.path.join(grandparent_directory_path, file_name)




    
    
    def correct_image(self,image):
        #================================
        # Corrige l'image en utilisant les paramètres de calibration
        # Args:
        #   image (ndarray): l'image à corriger
        # Returns:
        #   corrected_img (ndarray): l'image corrigée.
        #================================
    
        h, w = image.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(*self.get_calibration_params() , (w, h), 1, (w, h))

        # Corriger l'image en utilisant les paramètres de calibration
        corrected_img = cv2.undistort(image, *self.get_calibration_params() , None, new_mtx)
        script_dir = os.path.dirname(os.path.realpath(__file__))
        #cv2.imwrite(os.path.join(script_dir, "image__original.jpg"), image)
        #cv2.imwrite(os.path.join(script_dir, "image__corrected.jpg"), corrected_img)


        return corrected_img

    def get_aruco_marker_points(self, aruco_size):
        # Retourne les points 3D d'un marqueur ArUco
        half_size = aruco_size / 2.0
        return np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

    def detect_aruco_marker(self, image):
        # ==================================
        # Détecte les marqueurs ArUco dans l'image.
        # Args:
        #   image (ndarray): l'image à analyser.
        #   camera_matrix (ndarray): la matrice de calibration de la caméra.
        #   dist_coeffs (ndarray): les coefficients de distorsion de la caméra.
        # Returns:
        #   corners (ndarray): les coordonnées des marqueurs ArUco.
        #   ids (list): les identifiants des marqueurs ArUco.
        #   rvecs (ndarray): les vecteurs de rotation des marqueurs ArUco.
        #   tvecs (ndarray): les vecteurs de translation des marqueurs ArUco.
        # ==================================
        ids = None
        corners = None
        rejected = None

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray_image)

        if ids is not None:
            ids = [id[0] for id in ids]
            try:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.mtx,np.array([0,0,0,0,0]))
            except AttributeError:
                # Alternative method if estimatePoseSingleMarkers is not available
                rvecs = []
                tvecs = []
                for corner in corners:
                    success, rvec, tvec = cv2.solvePnP(self.get_aruco_marker_points(self.aruco_size), corner, self.mtx,[0,0,0,0,0])
                    if success:
                        rvecs.append(rvec)
                        tvecs.append(tvec)
                rvecs = np.array(rvecs)
                tvecs = np.array(tvecs)
            return corners, ids, rvecs, tvecs
        else:
            #print("Aucun marqueur ArUco n'a été détecté.")
            return None, None, None, None
    
    def detect_apriltag_marker(self, image):
        #================================
        # Détecte les marqueurs apriltag dans l'image.
        # Args:
        #   image (ndarray): l'image à analyser.
        # Returns:
        #   corners (ndarray): les coordonnées des marqueurs apriltag.
        #   ids (list): les identifiants des marqueurs apriltag.
        #   rvecs (ndarray): les vecteurs de rotation des marqueurs apriltag.
        #   tvecs (ndarray): les vecteurs de translation des marqueurs apriltag.
        #================================
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
        detection_results = self.detector.detect(gray_image)
        ids = [detection.tag_id for detection in detection_results]
        
        if ids is not None:
            for i in range(len(ids)):
                detection=detection_results[i]
                corners=detection.corners.reshape(1,4,2) 
                rvec, tvec ,_ = cv2.aruco.estimatePoseSingleMarkers(corners, self.tag_size,*self.get_calibration_params())
                return corners, ids, rvec, tvec

        return None, None, None, None
     
    def calculate_positions_in_gripper_frame(self, ids, rvecs, tvecs):
        """
        Calcule les positions des marqueurs dans le repère du gripper.
        
        Args:
            ids (list): les identifiants des marqueurs.
            rvecs (ndarray): les vecteurs de rotation des marqueurs.
            tvecs (ndarray): les vecteurs de translation des marqueurs.
        
        Returns:
            ids (list): les identifiants des marqueurs.
            rota (list): les vecteurs de rotation des marqueurs.
            transla (list): les vecteurs de translation des marqueurs.
        """
        if ids is not None:
            # Convertir les vecteurs de rotation en matrices de rotation
            rotation_matrices = [cv2.Rodrigues(rvec)[0] for rvec in rvecs]
            homogeneous_matrices = []
            
            # Construire des matrices homogènes pour chaque marqueur
            for i in range(len(rotation_matrices)):
                H = np.identity(4)
                H[0:3, 0:3] = rotation_matrices[i]
                H[0:3, 3] = tvecs[i].ravel()
                homogeneous_matrices.append(H)
            
            # Définir une matrice de transformation pour le repère du gripper
            transformation_matrix = np.array([
                [0, 0, 1, 0],
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 0, 1]
            ])

            new_homogeneous_matrices = []
            
            # Appliquer cette transformation aux matrices homogènes
            for H in homogeneous_matrices:
                new_H = transformation_matrix.dot(H)
                new_homogeneous_matrices.append(new_H)
            
            rota = []
            transla = []
            
            # Extraire les angles d'Euler et les vecteurs de translation des nouvelles matrices homogènes
            for H in new_homogeneous_matrices:
                euler_angles = np.flip(R.Rotation.from_matrix(H[0:3, 0:3]).as_euler('ZYX', degrees=False))
                rota.append(euler_angles)
                transla.append(H[0:3, 3])
            
            return ids, rota, transla
        else:
            return None, None, None
    
    def rotation_matrix_to_quaternion(self, matrix):
        """Convert a rotation matrix to a quaternion."""
        m = matrix
        trace = np.trace(m)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (m[2, 1] - m[1, 2]) * s
            qy = (m[0, 2] - m[2, 0]) * s
            qz = (m[1, 0] - m[0, 1]) * s
        else:
            if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
                qw = (m[2, 1] - m[1, 2]) / s
                qx = 0.25 * s
                qy = (m[0, 1] + m[1, 0]) / s
                qz = (m[0, 2] + m[2, 0]) / s
            elif m[1, 1] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
                qw = (m[0, 2] - m[2, 0]) / s
                qx = (m[0, 1] + m[1, 0]) / s
                qy = 0.25 * s
                qz = (m[1, 2] + m[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
                qw = (m[1, 0] - m[0, 1]) / s
                qx = (m[0, 2] + m[2, 0]) / s
                qy = (m[1, 2] + m[2, 1]) / s
                qz = 0.25 * s
        return np.array([qw, qx, qy, qz])

    def calculate_camera_position_in_aruco_frame(self, ids, rvecs, tvecs):
        if ids is not None:
            camera_positions = []
            camera_orientations = []

            for i in range(len(ids)):
                R_ca = cv2.Rodrigues(rvecs[i])[0]  # Rotation matrix from Aruco to camera
                t_ca = tvecs[i].reshape(3, 1)  # Translation vector from Aruco to camera

                # Invert the transformation
                R_ac = R_ca.T  # Rotation matrix from camera to Aruco
                t_ac = -R_ac @ t_ca  # Translation vector from camera to Aruco

                # Convert rotation matrix to quaternion
                quat = self.rotation_matrix_to_quaternion(R_ac)
                camera_orientations.append(quat)
                camera_positions.append(t_ac.flatten())

            return ids, camera_orientations, camera_positions
        else:
            return None, None, None

    def calculate_positions_in_world_frame(self, ids, rvecs, tvecs):
        #================================
        # Calcule les positions des marqueurs apriltag.
        # Args:
        #   ids (list): les identifiants des marqueurs apriltag.
        #   rvecs (ndarray): les vecteurs de rotation des marqueurs apriltag.
        #   tvecs (ndarray): les vecteurs de translation des marqueurs apriltag.
        # Returns:
        #   ids (list): les identifiants des marqueurs apriltag.
        #   rota (list): les vecteurs de rotation des marqueurs apriltag.
        #   transla (list): les vecteurs de translation des marqueurs apriltag.
        #================================
        if ids is not None:
            rotation_matrices = [cv2.Rodrigues(rvec)[0] for rvec in rvecs]
            homogeneous_matrices = []
            for i in range(len(rotation_matrices)):
                H = np.identity(4)
                H[0:3, 0:3] = rotation_matrices[i]
                H[0:3, 3] = tvecs[i].ravel()
                homogeneous_matrices.append(H)

            transformation_matrix = np.array([
                [0, 0, 1, 0],
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 0, 1]
            ])

            new_homogeneous_matrices = []
            for H in homogeneous_matrices:
                new_H = transformation_matrix.dot(H)
                new_homogeneous_matrices.append(new_H)

            rota = []
            transla = []
            for H in new_homogeneous_matrices:
                rot = R.Rotation.from_matrix(H[0:3, 0:3]).as_quat()
                rota.append(rot)
                transla.append(H[0:3, 3])

            return ids, rota, transla
        else:
            return None, None, None
        
        
    def calculate_positions_in_camera_frame(self, ids, rvecs, tvecs):
        if ids is not None:
            rotation_matrices = [cv2.Rodrigues(rvec)[0] for rvec in rvecs]
            rota = []
            
            for i in range (len(rotation_matrices)):
                rot = R.Rotation.from_matrix(rotation_matrices[i])
                rota.append(rot.as_quat())
            return ids, rota, tvecs[i]
        else:
            return None, None, None
        

    def detect_marker(self, image, tag_type, tag_size, tag_dict):
        #================================
        # Détecte les marqueurs (Aruco ou AprilTag) dans l'image.
        # Args:
        #   image (ndarray): l'image à analyser.
        #   marker_type (str): le type de marqueur à détecter ('aruco' ou 'apriltag').
        # Returns:
        #   corners (ndarray): les coordonnées des marqueurs.
        #   ids (list): les identifiants des marqueurs.
        #   rvecs (ndarray): les vecteurs de rotation des marqueurs.
        #   tvecs (ndarray): les vecteurs de translation des marqueurs.
        #================================
        
        if tag_type == 'aruco':
            self.set_aruco_params(tag_size,tag_dict)
            return self.detect_aruco_marker(image)
        elif tag_type == 'apriltag':
            self.set_apriltag_params(tag_size,tag_dict)
            return self.detect_apriltag_marker(image)
        else:
            print('Erreur: le type de marqueur doit être "aruco" ou "apriltag".')
            return None, None, None, None

class TagConfig:
    def __init__(self, tag_type, tag_size, tag_dict,tag_id = None, tag_rvec = None ,tag_tvec= None):
        self.tag_type = tag_type
        self.tag_size = tag_size
        self.tag_dict = tag_dict
        self.tag_id = tag_id
        self.tag_rvec = tag_rvec
        self.tag_tvec = tag_tvec
        #print(Fore.BLUE+ "Initialisation d'un marqueur : " + str(self.tag_type) + " " + str(self.tag_size) + " " + str(self.tag_dict)+ Style.RESET_ALL)
    
    #///////////////////////////////////////////////

    def set_tag_params(self, tag_type, tag_size, tag_dict):
        """
        Définit les paramètres du marqueur.
        
        Args:
            tag_type (str): Type du marqueur (par exemple, 'apriltag').
            tag_size (float): Taille du marqueur.
            tag_dict (str): Nom du dictionnaire utilisé pour le marqueur.
        """
        self.tag_type = tag_type
        self.tag_size = tag_size
        self.tag_dict = tag_dict
        

    def get_tag_params(self):
        """
        Retourne les paramètres du marqueur.
        
        Returns:
            tuple: Un tuple contenant le type, la taille et le dictionnaire du marqueur.
        """
        return self.tag_type, self.tag_size, self.tag_dict

    #///////////////////////////////////////////////

    def set_tag_id(self, tag_id):
        self.tag_id = tag_id

    def get_tag_id(self):
        return self.tag_id
    
    #///////////////////////////////////////////////

    def set_tag_rvec(self, tag_rvec):
        self.tag_rvec = tag_rvec
    
    def get_tag_rvec(self):
        return self.tag_rvec
    
    
    def set_tag_tvec(self, tag_tvec):
        self.tag_tvec = tag_tvec

    def get_tag_tvec(self):
        return self.tag_tvec
    
    #///////////////////////////////////////////////

class CameraConfig:
    def __init__(self, name,image_folder,mtx, dist):
        self.name = name
        self.image_folder = image_folder
        self.mtx = mtx
        self.dist = dist
        #print(Fore.BLUE+ "Initialisation de la caméra : ",name , Style.RESET_ALL)

    #///////////////////////////////////////////////

    def set_name(self, name):
        #================================
        # Définit le nom de la caméra.
        # Args:
        #   name (str): le nom de la caméra.
        #================================
        self.name = name
    
    def get_name(self):
        return self.name
    
    #///////////////////////////////////////////////

    def set_image_folder(self, image_folder):
        #================================
        # Définit le dossier contenant les images de la caméra.
        # Args:
        #   image_folder (str): le dossier contenant les images de la caméra.
        #================================
        self.image_folder = image_folder

    def get_image_folder(self):
        return self.image_folder
    
    #///////////////////////////////////////////////

    def set_calibration_params(self, mtx, dist):
        """
        Définit les paramètres de calibration de la caméra.
        
        Args:
            mtx (ndarray): Matrice de calibration de la caméra.
            dist (ndarray): Coefficients de distorsion de la caméra.
        """
        self.mtx = mtx
        self.dist = dist

    def get_calibration_params(self):
        """
        Retourne les paramètres de calibration de la caméra.
        
        Returns:
            tuple: Un tuple contenant la matrice de calibration et les coefficients de distorsion.
        """
        return self.mtx, self.dist



class TagDetection:
    def __init__(self, tag_config,camera_config):
        
        #self.camera_config = camera_config
        self.tag_detection_provider = TagPoseProvider()
        self.load_global_config(tag_config,camera_config)

        self.detector_callback()

    def load_global_config(self,tag_list,camera_list):
        #================================
        # Charge la configuration des tags ainsi que des cameras.
        # Args:
        #   tag_list: Liste des tags.
        #   camera_list: Liste des caméras.
        #================================

        self.tag_config = [TagConfig(tag['type'],tag['size'],tag['dict']) for tag in tag_list]
        
        #Créer un objet CameraConfig pour chaque caméra
        #Pour chaque caméra, on doit fournir :
        #                                    le nom de la caméra
        #                                    le chemin du dossier contenant les images (le chemin depuis la racine du projet ou depuis le dossier contenant le script python)
        #                                    la matrice de calibration et les coefficients de distorsion
        #self.camera_config = [CameraConfig(camera['name'],camera['images_folder'],*self.load_calib_data(self.tag_detection_provider.get_grandparent_file_path(camera['calibration_file'],'calib'))) for camera in camera_list]
        self.camera_config = [CameraConfig(camera['name'],camera['images_folder'],*self.load_calib_data(camera['calibration_file'])) for camera in camera_list]
       


    def load_calib_data(self,calibration_file):
        #================================
        # Charge les données de calibration depuis le fichier spécifié.
        # Args:
        #   calibration_file: Chemin vers le fichier de calibration.
        # Returns:
        #   matr: Matrice de calibration.
        #   disto: Coefficients de distorsion.
        #================================
        """
        dist = np.array(camera_infos_msg.D)
        mtx = np.array([camera_infos_msg.K[0:3], camera_infos_msg.K[3:6], camera_infos_msg.K[6:9]])
        """
        try:
            with open(calibration_file) as f:
                data = json.load(f)
            matr = data["mtx"]
            disto = data["dist"]
          
        except Exception as e:
            print("Erreur lors de la lecture du fichier de calibration :", e)
        return np.array(matr) , np.array(disto)


    def detector_callback(self):
        #================================
        # Callback appelée lors de la réception d'un message sur le topic de l'image.
        # Args:
        #   image_msg: Message ROS contenant l'image sur laquelle les marqueurs doivent être détectés.
        #================================
        
        for camera in self.camera_config:
            print('camera',camera)
            image_path = self.tag_detection_provider.get_grandparent_file_path(camera.image_folder)
            file_list = os.listdir(image_path)
            image_list = [file for file in file_list if file.endswith(".png")]
            self.tag_detection_provider.set_calibration_params(*camera.get_calibration_params())
            for img in image_list:
                cv_image = self.tag_detection_provider.correct_image(cv2.imread(os.path.join(image_path,img)))
                corners, ids, rvecs, tvecs = self.tag_detection_provider.detect_marker(cv_image,*self.tag_config[0].get_tag_params())  #TODO: enlever le [0] pour faire fonctionner avec plusieurs tags

                self.create_aruco_data_msg(*self.tag_detection_provider.calculate_positions(ids, rvecs, tvecs), cv_image)
        

        all_poses = []
        for i in range(len(id)):
            pose = {"id": id[i], "rvec": rvecs[i], "tvec": tvecs[i]}
            all_poses.append(pose)
        print('all_poses',all_poses)
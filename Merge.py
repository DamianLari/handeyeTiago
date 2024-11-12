import csv
import os
import argparse

class MergePose:
    def __init__(self, filename1, filename2):
        self.filename1 = filename1
        self.filename2 = filename2
        
    def isFormatGood(self, filename, required_fields):
        with open(filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            return all(field in reader.fieldnames for field in required_fields)
    
    def read_merged_poses(self,merged_filename):
        if not merged_filename:
            raise ValueError("Le nom du fichier fusionné n'est pas défini")
        
        required_fields = [
            'image_file', 'time', 'tag_id', 'tag_tx', 'tag_ty', 'tag_tz', 
            'tag_rx', 'tag_ry', 'tag_rz', 'gripper_tx', 'gripper_ty', 
            'gripper_tz', 'gripper_rx', 'gripper_ry', 'gripper_rz'
        ]
        
        if not self.isFormatGood(merged_filename, required_fields):
            raise ValueError("Le format du fichier CSV fusionné n'est pas correct")
        
        merged_poses = {}
        with open(merged_filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                merged_poses[row['image_file']] = {
                    'tag_id': int(row['tag_id']),
                    'tag_pose': {
                        'tx': float(row['tag_tx']),
                        'ty': float(row['tag_ty']),
                        'tz': float(row['tag_tz']),
                        'rx': float(row['tag_rx']),
                        'ry': float(row['tag_ry']),
                        'rz': float(row['tag_rz'])
                    },
                    'gripper_pose': {
                        'time':int(row['time']),
                        'tx': float(row['gripper_tx']),
                        'ty': float(row['gripper_ty']),
                        'tz': float(row['gripper_tz']),
                        'rx': float(row['gripper_rx']),
                        'ry': float(row['gripper_ry']),
                        'rz': float(row['gripper_rz'])
                    }
                }
        return merged_poses
    
    
    def merge_poses(self):
        required_fields1 = ['time','tx', 'ty', 'tz', 'rx', 'ry', 'rz']
        required_fields2 = ['image_file', 'tag_id', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz']
        """
        if not (self.isFormatGood(self.filename1, required_fields1) and self.isFormatGood(self.filename2, required_fields2)):
            raise ValueError("Le format des fichiers CSV n'est pas correct")
        """
        poses1 = self.read_pose_csv(self.filename1)
        poses2 = self.read_image_csv(self.filename2)

        merged_poses = self.associate_poses(poses1, poses2)
        return merged_poses

    def read_pose_csv(self, filename):
        poses = []
        with open(filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            if not all(field in reader.fieldnames for field in ['tx', 'ty', 'tz', 'rx', 'ry', 'rz']):
                raise ValueError(f"Fichier {filename} n'a pas les champs nécessaires : 'tx', 'ty', 'tz', 'rx', 'ry', 'rz'")
            for row in reader:
                pose = {
                    'time': int(row['time']),
                    'tx': float(row['tx']),
                    'ty': float(row['ty']),
                    'tz': float(row['tz']),
                    'rx': float(row['rx']),
                    'ry': float(row['ry']),
                    'rz': float(row['rz'])
                }
                poses.append(pose)
        return poses

    def read_image_csv(self, filename):
        images = []
        with open(filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            
            if not all(field in reader.fieldnames for field in ['image_file', 'tag_id', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz']):
                raise ValueError(f"Fichier {filename} n'a pas les champs nécessaires : 'image_file', 'tag_id', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz'")
            for row in reader:
                image_data = {
                    'image_file': row['image_file'].strip(), 
                    'tag_id': int(row['tag_id']),
                    'tx': float(row['tx']),
                    'ty': float(row['ty']),
                    'tz': float(row['tz']),
                    'rx': float(row['rx']),
                    'ry': float(row['ry']),
                    'rz': float(row['rz'])
                }
                images.append(image_data)
        return images

    def associate_poses(self, poses, images):
        associated_poses = {}
        for image in images:
            image_basename = os.path.splitext(os.path.basename(image['image_file']))[0]
            image_number_str = ''.join(filter(str.isdigit, image_basename))
            try:
                image_number = int(image_number_str)
            except ValueError:
                print(f"Impossible de convertir {image_basename} en nombre")
                continue
            
            if image_number - 1 < len(poses):  
                gripper_pose = poses[image_number - 1] 
                image_file_with_extension = f"{image_basename}.jpg"  
                associated_poses[image_file_with_extension] = {
                    'tag_id': image['tag_id'],
                    'gripper_pose': gripper_pose,
                    'tag_pose': {
                        'tx': image['tx'],
                        'ty': image['ty'],
                        'tz': image['tz'],
                        'rx': image['rx'],
                        'ry': image['ry'],
                        'rz': image['rz']
                    }
                }
            else:
                print(f"Aucune pose de gripper correspondante pour l'image {image['image_file']}")
        return associated_poses
    
    def save_to_csv(self, output_filename):
        if os.path.exists(output_filename):
            os.remove(output_filename)
        merged_poses = self.merge_poses()
        with open(output_filename, 'w', newline='') as csvfile:
            fieldnames = ['image_file', 'time', 'tag_id', 'tag_tx', 'tag_ty', 'tag_tz', 'tag_rx', 'tag_ry', 'tag_rz',
                          'gripper_tx', 'gripper_ty', 'gripper_tz', 'gripper_rx', 'gripper_ry', 'gripper_rz']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for image_file, data in merged_poses.items():
                row = {
                    'image_file': image_file,
                    'time' : data['gripper_pose']['time'],
                    'tag_id': data['tag_id'],
                    'tag_tx': data['tag_pose']['tx'],
                    'tag_ty': data['tag_pose']['ty'],
                    'tag_tz': data['tag_pose']['tz'],
                    'tag_rx': data['tag_pose']['rx'],
                    'tag_ry': data['tag_pose']['ry'],
                    'tag_rz': data['tag_pose']['rz'],
                    'gripper_tx': data['gripper_pose']['tx'],
                    'gripper_ty': data['gripper_pose']['ty'],
                    'gripper_tz': data['gripper_pose']['tz'],
                    'gripper_rx': data['gripper_pose']['rx'],
                    'gripper_ry': data['gripper_pose']['ry'],
                    'gripper_rz': data['gripper_pose']['rz']
                }
                writer.writerow(row)
                
    def get_certain_tag_pose(self, image_file):
        merged_poses = self.merge_poses()
        #print("Clés disponibles dans merged_poses:", list(merged_poses.keys())) 
        image_file = image_file.strip()  
        if image_file not in merged_poses:
            raise KeyError(f"Le fichier image '{image_file}' n'existe pas dans merged_poses")
        return merged_poses[image_file]['tag_pose']
    
    def get_certain_gripper_pose(self, image_file):
        merged_poses = self.merge_poses()
        #print("Clés disponibles dans merged_poses:", list(merged_poses.keys()))  
        image_file = image_file.strip()  
        if image_file not in merged_poses:
            raise KeyError(f"Le fichier image '{image_file}' n'existe pas dans merged_poses")
        return merged_poses[image_file]['gripper_pose']


def main():
    parser = argparse.ArgumentParser(description='Estimation de la pose d\'un tag et enregistrement dans un fichier CSV')
    parser.add_argument('--gripper_pose_csv', type=str, help='Fichier CSV contenant les poses du gripper')
    parser.add_argument('--tag_pose_csv', type=str, help='Fichier CSV contenant les poses du tag')
    parser.add_argument('--output_csv', type=str, help='Fichier CSV de sortie')

    args = parser.parse_args()

    merger = MergePose( args.gripper_pose_csv, args.tag_pose_csv)
    merger.save_to_csv(args.output_csv)
    #print(merger.get_certain_tag_pose('image_0001.jpg'))

if __name__ == "__main__":
    main()

import csv
import os
import argparse

class MergePose:
    def __init__(self, filename1, filename2):
        self.filename1 = filename1  # CSV du gripper
        self.filename2 = filename2  # CSV du tag
        
    def isFormatGood(self, filename, required_fields):
        with open(filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            return all(field in reader.fieldnames for field in required_fields)
    
    def read_csv_with_image(self, filename, fields):
        """ Lit un CSV et retourne un dictionnaire avec image_file comme clé """
        data = {}
        with open(filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            if not all(field in reader.fieldnames for field in fields):
                raise ValueError(f"Fichier {filename} n'a pas les champs nécessaires : {fields}")
            for row in reader:
                image_file = row['image_file'].strip()
                data[image_file] = {field: float(row[field]) if field not in ['image_file', 'tag_id'] else row[field] for field in fields}
                if 'tag_id' in fields:
                    data[image_file]['tag_id'] = int(row['tag_id'])
        return data

    def merge_poses(self):
        # Champs requis pour chaque fichier CSV
        required_fields_gripper = ['image_file', 'time', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz']
        required_fields_tag = ['image_file', 'tag_id', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz']
        
        # Lecture des données des fichiers CSV
        gripper_poses = self.read_csv_with_image(self.filename1, required_fields_gripper)
        tag_poses = self.read_csv_with_image(self.filename2, required_fields_tag)
        
        # Fusion des poses
        merged_poses = {}
        for image_file, gripper_pose in gripper_poses.items():
            # Debugging : vérifier si chaque image du gripper a une correspondance dans les tags
            image_name_no_ext, ext = os.path.splitext(image_file)
            if image_file in tag_poses:
                tag_pose = tag_poses[image_file]
                merged_poses[image_file] = {
                    'time': gripper_pose['time'],
                    'tag_id': tag_pose['tag_id'],
                    'tag_pose': {
                        'tx': tag_pose['tx'],
                        'ty': tag_pose['ty'],
                        'tz': tag_pose['tz'],
                        'rx': tag_pose['rx'],
                        'ry': tag_pose['ry'],
                        'rz': tag_pose['rz']
                    },
                    'gripper_pose': {
                        'tx': gripper_pose['tx'],
                        'ty': gripper_pose['ty'],
                        'tz': gripper_pose['tz'],
                        'rx': gripper_pose['rx'],
                        'ry': gripper_pose['ry'],
                        'rz': gripper_pose['rz']
                    }
                }
            else:
                print(f"[INFO] Aucun tag correspondant pour l'image '{image_file}'")
        return merged_poses
    
    def save_to_csv(self, output_filename):
        merged_poses = self.merge_poses()
        
        if not merged_poses:
            print("[AVERTISSEMENT] Aucune correspondance trouvée entre les poses du gripper et du tag. Le fichier CSV de sortie sera vide.")
            return

        # Enregistrement dans le fichier CSV de sortie
        if os.path.exists(output_filename):
            os.remove(output_filename)

        with open(output_filename, 'w', newline='') as csvfile:
            fieldnames = ['image_file', 'time', 'tag_id', 'tag_tx', 'tag_ty', 'tag_tz', 'tag_rx', 'tag_ry', 'tag_rz',
                          'gripper_tx', 'gripper_ty', 'gripper_tz', 'gripper_rx', 'gripper_ry', 'gripper_rz']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for image_file, data in merged_poses.items():
                row = {
                    'image_file': image_file,
                    'time': data['time'],
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
        print(f"[INFO] Fichier CSV de sortie '{output_filename}' créé avec succès.")

def main():
    parser = argparse.ArgumentParser(description='Fusion des poses du tag et du gripper en fonction du nom de l\'image')
    parser.add_argument('--gripper_pose_csv', type=str, help='Fichier CSV contenant les poses du gripper')
    parser.add_argument('--tag_pose_csv', type=str, help='Fichier CSV contenant les poses du tag')
    parser.add_argument('--output_csv', type=str, help='Fichier CSV de sortie')

    args = parser.parse_args()

    merger = MergePose(args.gripper_pose_csv, args.tag_pose_csv)
    merger.save_to_csv(args.output_csv)

if __name__ == "__main__":
    main()



import xml.etree.ElementTree as ET

def load_sdf_file(file_path):
    # Chargement et parsing du fichier XML
    tree = ET.parse(file_path)
    root = tree.getroot()
    
    # Initialisation des données
    sdf_data = {
        "world": {},
        "models": []
    }
    
    # Extraction des informations de base du monde
    world = root.find('world')
    if world is not None:
        # Gravité
        gravity = world.find('gravity')
        if gravity is not None:
            sdf_data["world"]["gravity"] = gravity.text.strip()
        
        # Lumière(s)
        lights = []
        for light in world.findall('light'):
            light_data = {
                "name": light.get('name'),
                "type": light.get('type'),
                "pose": light.find('pose').text.strip() if light.find('pose') is not None else None,
                "diffuse": light.find('diffuse').text.strip() if light.find('diffuse') is not None else None
            }
            lights.append(light_data)
        sdf_data["world"]["lights"] = lights
    
    # Extraction des modèles
    for model in world.findall('model'):
        model_data = {
            "name": model.get('name'),
            "pose": model.find('pose').text.strip() if model.find('pose') is not None else None,
            "links": []
        }
        
        # Extraction des liens dans chaque modèle
        for link in model.findall('link'):
            link_data = {
                "name": link.get('name'),
                "mass": None,
                "collision": None,
                "visual": None
            }
            
            # Masse de l'élément
            inertial = link.find('inertial')
            if inertial is not None:
                mass = inertial.find('mass')
                if mass is not None:
                    link_data["mass"] = mass.text.strip()
            
            # Collision shape
            collision = link.find('collision')
            if collision is not None:
                collision_geometry = collision.find('geometry')
                if collision_geometry is not None:
                    box = collision_geometry.find('box')
                    if box is not None:
                        link_data["collision"] = box.find('size').text.strip() if box.find('size') is not None else None
            
            # Visual shape
            visual = link.find('visual')
            if visual is not None:
                visual_geometry = visual.find('geometry')
                if visual_geometry is not None:
                    box = visual_geometry.find('box')
                    if box is not None:
                        link_data["visual"] = box.find('size').text.strip() if box.find('size') is not None else None
            
            model_data["links"].append(link_data)
        
        sdf_data["models"].append(model_data)
    
    return sdf_data

file_path = "aruco11.sdf"
sdf_data = load_sdf_file(file_path)

print(sdf_data)

# HandEye Calibration pour Transformation Tête/Base
Article en cours d'écriture
Le dernier rapport écrit date du 07/11/24 et est dicponible [ici](Rapport07_11_24.pdf)



## Aperçu

Ce projet a pour objectif d'estimer la transformation entre la base d'un robot et la tête à l'aide de la calibration HandEye. En utilisant des données collectées via des simulations sous Gazebo et des poses détectées par un ArUco, ce projet offre une solution pour aligner efficacement les systèmes de coordonnées de la base et de la tête.

Le projet est conçu pour les chercheurs, ingénieurs en robotique, et toute personne intéressée par l'intégration de robots et la perception visuelle. Il propose des fonctionnalités telles que :

- Extraction des poses de la base et des ArUco dans un environnement simulé.
- Utilisation de la méthode de calibration HandEye pour déterminer la transformation entre différents repères.
- Visualisation et comparaison des résultats estimés avec les valeurs de référence.

## Fonctionnalités

- **Extraction des poses sous Gazebo** : Collecte des poses via des simulations sous Gazebo utilisant des marqueurs ArUco pour simuler les positions de la caméra.
- **Calibration HandEye** : Implémente une calibration HandEye utilisant la méthode RANSAC pour déterminer la transformation optimale entre la base et la caméra.
- **Comparaison des Transformations** : Génération de graphiques et tableaux pour comparer les valeurs estimées des translations et rotations avec les valeurs réelles.

## Pré-requis et Installation

Ce projet nécessite les outils et bibliothèques suivants :

- **Gazebo** : Simulateur pour robots permettant de reproduire des scénarios dans un environnement virtuel.
- **ROS (Robot Operating System)** : Pour la communication avec Gazebo et la collecte des données.
- **Python 3** : Langage de programmation principal pour l'analyse et le traitement des données.
- **OpenCV** : Utilisé pour la détection des marqueurs ArUco et la calibration HandEye.


## Utilisation

Après avoir installé les dépendances, vous pouvez commencer à utiliser ce projet pour calibrer le système de coordonnées entre la base et la tête d'un robot. Voici un guide de base pour l'utilisation :

1. **Lancer Gazebo** : Assurez-vous que le simulateur Gazebo est correctement configuré et que les marqueurs ArUco sont visibles dans la scène.
2. **Exécuter la collecte des données** : Utilisez les scripts fournis pour extraire les poses du marqueur et des objets pertinents (par exemple, la caméra).
3. **Lancer la calibration HandEye** :Le script va exécuter la calibration HandEye sur les données collectées.
4. **Visualiser les résultats** : Les graphiques générés montreront la comparaison entre les valeurs estimées et les valeurs réelles des translations et rotations.



## Remerciements

- **Gazebo** - Simulateur permettant de reproduire des environnements de robotique de manière réaliste.
- **OpenCV** - Utilisé pour la détection des marqueurs ArUco.
- **ROS** - Fournit la structure de communication entre les différents composants du projet.




import csv
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
import numpy as np
class AnalyseResults:
    def __init__(self, output_file_path):
        self.output_file_path = output_file_path
        self.results = self.read_results()

    def load_results(self):
        results = {}
        with open(self.output_file_path, mode='r') as csvfile:
            csv_reader = csv.DictReader(csvfile)
            for row in csv_reader:
                key = row['armAndTag']
                results[key] = {
                    'Rx': float(row['cRb_x']),
                    'Ry': float(row['cRb_y']),
                    'Rz': float(row['cRb_z']),
                    'Tx': float(row['cTb_x']),
                    'Ty': float(row['cTb_y']),
                    'Tz': float(row['cTb_z']),
                    'mean_error': float(row['mean_error']),
                    'stdev_error':float(row['stdev_error']),
                    'mean_3D_error': float(row['mean_3D_error']),
                    'stdev_3D_error': float(row['stdev_3D_error'])
                }
        return results

    def read_results(self):
        results = self.load_results()
        #for key in results:
            #print(f"{key}: {results[key]}")
        return results

    def get_results(self):
        return self.results

    def results_to_dataframe(self):
        results_dict = self.get_results()
        data = {
            'armAndTag': [],
            'Rx': [], 'Ry': [], 'Rz': [],
            'Tx': [], 'Ty': [], 'Tz': [],
            'mean_error': [], 'stdev_error': [],
            'mean_3D_error': [], 'stdev_3D_error': []
        }
        for key, values in results_dict.items():
            data['armAndTag'].append(key)
            data['Rx'].append(values['Rx'])
            data['Ry'].append(values['Ry'])
            data['Rz'].append(values['Rz'])
            data['Tx'].append(values['Tx'])
            data['Ty'].append(values['Ty'])
            data['Tz'].append(values['Tz'])
            data['mean_error'].append(values['mean_error'])
            data['stdev_error'].append(values['stdev_error'])
            data['mean_3D_error'].append(values['mean_3D_error'])
            data['stdev_3D_error'].append(values['stdev_3D_error'])
        df = pd.DataFrame(data)
        return df
    
    def visualize_3D_error(self, df):
        """
        Visualise les erreurs 3D.
        """
        plt.figure(figsize=(12, 6))
        plt.plot(df['armAndTag'], df['3D_error'], marker='o', label='Erreur 3D', color="purple")
        plt.title('Erreurs 3D')
        plt.xlabel('Arm and Tag')
        plt.ylabel('Erreur 3D (metres)')
        plt.legend()
        plt.xticks(rotation=45)  
        plt.tight_layout()
        plt.show()

    def calculate_3D_error(self, df):
        """
        Calcule l'erreur 3D pour chaque entrée du DataFrame.
        """
        df['3D_error'] = np.sqrt(df['Tx']**2 + df['Ty']**2 + df['Tz']**2)
        return df
    
    def visualize_data_histogram(self, df):
        fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(12, 8))
        translation_colors = ['red', 'green', 'blue']
        rotation_colors = ['red', 'green', 'blue']
        
        df.plot(x='armAndTag', y=['Tx', 'Ty', 'Tz'], kind='line',color=translation_colors, ax=axes[0][0])
        axes[0][0].set_title('Valeur Translation')
        axes[0][0].set_ylabel('Translation (metres)')

        df.plot(x='armAndTag', y=['Tx', 'Ty', 'Tz'], kind='box', ax=axes[0][1])
        axes[0][1].set_title('Valeur Translation')
        axes[0][1].set_ylabel('Translation (metres)')

        df.plot(x='armAndTag', y=['Rx', 'Ry', 'Rz'], kind='line', color=rotation_colors, ax=axes[1][0])
        axes[1][0].set_title('Valeur rotation')
        axes[1][0].set_ylabel('Rotation (radians)')
        
        df.plot(x='armAndTag', y=['Rx', 'Ry', 'Rz'], kind='box', ax=axes[1][1])
        axes[1][1].set_title('Valeur rotation')
        axes[1][1].set_ylabel('Rotation (radians)')

        plt.tight_layout()
        plt.show()

    def visualize_data_matplotlib(self, df):
        fig1 = plt.figure(figsize=(12, 8))
        ax1 = fig1.add_subplot(111, projection='3d')
        ax1.scatter(df['Tx'], df['Ty'], df['Tz'], c='b', marker='o')
        ax1.set_title('Translation Values')
        ax1.set_xlabel('Tx (meters)')
        ax1.set_ylabel('Ty (meters)')
        ax1.set_zlabel('Tz (meters)')
        for i, txt in enumerate(df['armAndTag']):
            ax1.text(df['Tx'][i], df['Ty'][i], df['Tz'][i], txt, size=8, zorder=1, color='k')
        plt.tight_layout()
        plt.show()

        fig2 = plt.figure(figsize=(12, 8))
        ax2 = fig2.add_subplot(111, projection='3d')
        ax2.scatter(df['Rx'], df['Ry'], df['Rz'], c='r', marker='^')
        ax2.set_title('Rotation Values')
        ax2.set_xlabel('Rx (radians)')
        ax2.set_ylabel('Ry (radians)')
        ax2.set_zlabel('Rz (radians)')
        for i, txt in enumerate(df['armAndTag']):
            ax2.text(df['Rx'][i], df['Ry'][i], df['Rz'][i], txt, size=8, zorder=1, color='k')
        plt.tight_layout()
        plt.show()

    def visualize_data_lines(self, df):
        plt.figure(figsize=(12, 6))
        plt.plot(df['armAndTag'], df['Tx'], marker='o', label='Tx (meters)', color="red")
        plt.plot(df['armAndTag'], df['Ty'], marker='o', label='Ty (meters)', color="green")
        plt.plot(df['armAndTag'], df['Tz'], marker='o', label='Tz (meters)', color="blue")
        plt.title('Translation Values')
        plt.xlabel('Arm and Tag')
        plt.ylabel('Translation (meters)')
        plt.legend()
        plt.xticks(rotation=45)  
        plt.tight_layout()
        plt.show()

        plt.figure(figsize=(12, 6))
        plt.plot(df['armAndTag'], df['Rx'], marker='o', label='Rx (radians)', color="red")
        plt.plot(df['armAndTag'], df['Ry'], marker='o', label='Ry (radians)', color="green")
        plt.plot(df['armAndTag'], df['Rz'], marker='o', label='Rz (radians)', color="blue")
        plt.title('Rotation Values')
        plt.xlabel('Arm and Tag')
        plt.ylabel('Rotation (radians)')
        plt.legend()
        plt.xticks(rotation=45)  
        plt.tight_layout()
        plt.show()

    def visualize_data_scatter(self, df):
        plt.figure(figsize=(12, 6))
        plt.scatter(df['armAndTag'], df['Tx'], label='Tx (meters)', marker='o')
        plt.scatter(df['armAndTag'], df['Ty'], label='Ty (meters)', marker='s')
        plt.scatter(df['armAndTag'], df['Tz'], label='Tz (meters)', marker='^')
        plt.title('Translation Values')
        plt.xlabel('Arm and Tag')
        plt.ylabel('Translation (meters)')
        plt.legend()
        plt.xticks(rotation=45)  
        plt.tight_layout()
        plt.show()
    
        plt.figure(figsize=(12, 6))
        plt.scatter(df['armAndTag'], df['Rx'], label='Rx (radians)', marker='o')
        plt.scatter(df['armAndTag'], df['Ry'], label='Ry (radians)', marker='s')
        plt.scatter(df['armAndTag'], df['Rz'], label='Rz (radians)', marker='^')
        plt.title('Rotation Values')
        plt.xlabel('Arm and Tag')
        plt.ylabel('Rotation (radians)')
        plt.legend()
        plt.xticks(rotation=45)  
        plt.tight_layout()
        plt.show()

    def visualize_rankings(self, df):
        plt.figure(figsize=(12, 6))
        df_sorted = df.sort_values(by='mean_error')
        plt.bar(df_sorted['armAndTag'], df_sorted['mean_error'], color='skyblue')
        plt.title('Classement des Erreurs de Reprojection Moyennes')
        plt.xlabel('Arm and Tag')
        plt.ylabel('Erreur de Reprojection Moyenne')
        plt.xticks(rotation=45) 
        plt.tight_layout()
        plt.show()

    def visualize_box_plot(self, df):
        fig, axes = plt.subplots(nrows=1, ncols=2, figsize=(14, 7))
        
        df_left = df[df['armAndTag'].str.contains('Left')]
        axes[0].boxplot([df_left['Tx'], df_left['Ty'], df_left['Tz']], labels=['Tx', 'Ty', 'Tz'])
        axes[0].set_title('Bras gauche')

        df_right = df[df['armAndTag'].str.contains('Right')]
        axes[1].boxplot([df_right['Tx'], df_right['Ty'], df_right['Tz']], labels=['Tx', 'Ty', 'Tz'])
        axes[1].set_title('Bras droit')

        plt.tight_layout()
        plt.show()

    def plot_errors_vs_inliers(self,df):
        plt.figure(figsize=(10, 6))
        plt.plot(df['armAndTag'], df['mean_error'], marker='o', label='Mean Reprojection Error', color='blue')
        plt.plot(df['armAndTag'], df['mean_3D_error'], marker='o', label='Mean 3D Error', color='red')
        plt.title('Errors vs Inliers')
        plt.xlabel('Inliers (ArmAndTag)')
        plt.ylabel('Error')
        plt.legend()
        plt.grid(True)
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()

    def visualize_3D_error(self, df):
        """
        Visualise les erreurs 3D.
        """
        plt.figure(figsize=(12, 6))
        plt.plot(df['armAndTag'], df['mean_3D_error'], marker='o', label='Erreur 3D Moyenne', color="purple")
        plt.fill_between(df['armAndTag'], 
                         df['mean_3D_error'] - df['stdev_3D_error'], 
                         df['mean_3D_error'] + df['stdev_3D_error'], 
                         color="lavender", alpha=0.5)
        plt.title('Erreurs 3D')
        plt.xlabel('Arm and Tag')
        plt.ylabel('Erreur 3D (metres)')
        plt.legend()
        plt.xticks(rotation=45)  
        plt.tight_layout()
        plt.show()

    def plot_bar_translation_rotation(self,df):
        df_sorted = df.sort_values(by='armAndTag')
        df_sorted.plot(x='armAndTag', y=['Tx', 'Ty', 'Tz'], kind='bar', figsize=(12, 6))
        plt.title('Translation Values')
        plt.xlabel('Inliers (ArmAndTag)')
        plt.ylabel('Translation (meters)')
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()

        df_sorted.plot(x='armAndTag', y=['Rx', 'Ry', 'Rz'], kind='bar', figsize=(12, 6))
        plt.title('Rotation Values')
        plt.xlabel('Inliers (ArmAndTag)')
        plt.ylabel('Rotation (radians)')
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()

    def plot_box_plot(self,df):
        fig, axes = plt.subplots(nrows=1, ncols=2, figsize=(14, 7))

        df.plot(kind='box', y=['Tx', 'Ty', 'Tz'], ax=axes[0], title='Translation Distribution')
        axes[0].set_ylabel('Translation (meters)')

        df.plot(kind='box', y=['Rx', 'Ry', 'Rz'], ax=axes[1], title='Rotation Distribution')
        axes[1].set_ylabel('Rotation (radians)')

        plt.tight_layout()
        plt.show()

    def plot_3d_translation(self,df):
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(df['Tx'], df['Ty'], df['Tz'], c='blue', marker='o')

        ax.set_title('3D Translation Plot')
        ax.set_xlabel('Tx (meters)')
        ax.set_ylabel('Ty (meters)')
        ax.set_zlabel('Tz (meters)')

        for i, txt in enumerate(df['armAndTag']):
            ax.text(df['Tx'][i], df['Ty'][i], df['Tz'][i], txt, size=8, zorder=1, color='k')

        plt.tight_layout()
        plt.show()

    def plot_correlation_heatmap(self, df):
        # Sélectionner uniquement les colonnes numériques
        numeric_df = df.select_dtypes(include=[np.number])
        
        plt.figure(figsize=(10, 8))
        sns.heatmap(numeric_df.corr(), annot=True, cmap='coolwarm', vmin=-1, vmax=1)
        plt.title('Correlation Heatmap')
        plt.show()

    def plot_multiple_subplots(self,df):
        fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(14, 10))
        translation_colors = ['red', 'green', 'blue']
        rotation_colors = ['red', 'green', 'blue']
        df.plot(x='armAndTag', y=['mean_error'], ax=axes[0][0], marker='o', title='Mean Reprojection Error')
        df.plot(x='armAndTag', y=['mean_3D_error'], ax=axes[0][1], marker='o', title='Mean 3D Error')
        df.plot(x='armAndTag', y=['Tx', 'Ty', 'Tz'], ax=axes[1][0], marker='o', color=translation_colors, title='Translation')
        df.plot(x='armAndTag', y=['Rx', 'Ry', 'Rz'], ax=axes[1][1], marker='o', color=rotation_colors,title='Rotation')

        for ax in axes.flat:
            ax.set_xlabel('ArmAndTag')
            ax.set_ylabel('Values')
            ax.grid(True)
            ax.legend()

        plt.tight_layout()
        plt.show()

output_file_path = 'calib_results.csv'
analyser = AnalyseResults(output_file_path)

# Convert results to DataFrame
df = analyser.results_to_dataframe()

# Visualize the data
#analyser.visualize_data_histogram(df)
#analyser.visualize_3D_error(df)
#analyser.plot_errors_vs_inliers(df)
#analyser.plot_bar_translation_rotation(df)
#analyser.plot_box_plot(df)
#analyser.plot_3d_translation(df)
analyser.plot_multiple_subplots(df)
#analyser.plot_correlation_heatmap(df)

#analyser.visualize_rankings(df)
#analyser.visualize_box_plot(df)
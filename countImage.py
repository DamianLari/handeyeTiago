import os

def count_files_and_get_120th(directory_path):
    all_files = [f for f in os.listdir(directory_path) if os.path.isfile(os.path.join(directory_path, f))]
    total_files = len(all_files)
    
    print(f"Total number of files in the directory: {total_files}")
    
    if total_files >= 120:
        file_120th = all_files[119]  
        print(f"The 120th file is: {file_120th}")
        return file_120th
    else:
        print("There are less than 120 files in the directory.")
        return None

# Example usage
directory_path = "path/to/your/directory"
count_files_and_get_120th(directory_path)

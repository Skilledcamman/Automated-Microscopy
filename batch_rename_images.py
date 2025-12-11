import os

# Set your folder path and new base name
folder = r'C:\Users\ahmed\Desktop\microscope\snapshots'
new_base = 'camellia_pollen'

# List all jpg files in the folder
files = [f for f in os.listdir(folder) if f.lower().endswith('.jpg')]
files.sort()  # Optional: sort for consistent order

for idx, filename in enumerate(files, 1):
    old_path = os.path.join(folder, filename)
    new_name = f'{new_base}_{idx:04d}.jpg'
    new_path = os.path.join(folder, new_name)
    os.rename(old_path, new_path)
    print(f'Renamed: {filename} -> {new_name}')

print('Batch rename complete.')

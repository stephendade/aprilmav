#!/usr/bin/env python3

import os
import argparse
from PIL import Image

# Set up argument parser
parser = argparse.ArgumentParser(description='Rotate all images in a specified folder by 180 degrees.')
parser.add_argument('folder_path', type=str, help='Path to the folder containing the images')

# Parse the arguments
args = parser.parse_args()

# Get the folder path from the arguments
folder_path = args.folder_path

# Loop through all files in the folder
for filename in os.listdir(folder_path):
    if filename.endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
        print(f"Rotating {filename}...")
        # Open an image file
        with Image.open(os.path.join(folder_path, filename)) as img:
            # Rotate the image by 180 degrees
            rotated_img = img.rotate(180)
            # Save the rotated image back to the same file
            rotated_img.save(os.path.join(folder_path, filename))

print("All images have been rotated by 180 degrees.")

from PIL import Image
import argparse

import os

parser = argparse.ArgumentParser(description='A test program.')
parser.add_argument("path_folder", help="path to image folder")

path = parser.parse_args()

print(path.path_folder)

i = 0

for filename in os.listdir(str(path.path_folder)):
   im1 = Image.open(filename)
   im1.save(i + ".png")
   i = i + 1
   


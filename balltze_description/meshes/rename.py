import os
import shutil

files = os.listdir(os.path.dirname(os.path.realpath(__file__)))
input_file_name = 'imput.stl'
files.remove(input_file_name)
files.remove(os.path.basename(__file__))

for file in files:
  shutil.copyfile(input_file_name, file)
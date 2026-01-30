import yaml
from glob import glob
import os


this_path = os.path.abspath(__file__) 
this_dir = os.path.dirname(this_path)
base_dir = os.path.dirname(this_dir)
param_dir = os.path.join(base_dir,"params")
param_filepath = os.path.join(param_dir,"robot_params.yaml")

with open(param_filepath,"r") as file:
    config = yaml.safe_load(file)
print(config["central"]["ros__parameters"]["N"])
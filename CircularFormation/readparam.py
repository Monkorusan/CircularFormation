import yaml
from glob import glob
import os
import rclpy
from rclpy.node import Node






class ParamReader(Node):

    this_path = os.path.abspath(__file__) 
    this_dir = os.path.dirname(this_path)
    base_dir = os.path.dirname(this_dir)
    param_dir = os.path.join(base_dir,"params")
    param_filepath = os.path.join(param_dir,"robot_params.yaml")

    def __init__(self,node_name:str,param_file):
        """args expects namespace in yaml file"""
        self.param_value_dict = dict()
        self.param_callback_dict = dict()
        self.param_type_dict = dict()
        with open(self.param_filepath,"r") as file:
            config = yaml.safe_load(file)
        print(f"ParamReader is reading {self.param_filepath}")
    
    def load_parameters_from_file(self, param_file: str, package_name: str, dir_name: str = 'params'):
        pass    

    def get_parameter_value(self, name:str):
        """returns value of parameter in yaml file"""
        if name in self.param_value_dict.keys():
            return self.param_value_dict[name]
        else:
            return self.get_parameter(name).value #ROS2 python 

    def process_tree_dict(self, tree_dict:dict , ancestor_list:list):


config = ParamReader("Central","ros__parameters")
print(config.read(["N"])) #6
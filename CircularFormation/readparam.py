import yaml
from glob import glob
import os
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from functools import partial
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

class ParamReader(Node):

    this_path = os.path.abspath(__file__) 
    this_dir = os.path.dirname(this_path)
    base_dir = os.path.dirname(this_dir)
    param_dir = os.path.join(base_dir,"params")
    param_filepath = os.path.join(param_dir,"robot_params.yaml")

    def __init__(self, node_name: str = "param_reader"):
        """args expects namespace in yaml file"""
        super().__init__(node_name)
        self.param_value_dict = dict()
        self.param_callback_dict = dict()
        self.param_type_dict = dict()
        with open(self.param_filepath,"r") as file:
            config = yaml.safe_load(file)
        print(f"ParamReader is reading {self.param_filepath}")
    
    def load_parameters_from_file(self, param_file: str, package_name: str, dir_name: str = 'params'):
        """bypasses the hassle of having to type config[str][str] when reading a config from a YAML file"""
        try:
            param_dir = os.path.join(get_package_share_directory(package_name), dir_name, param_file)
        except PackageNotFoundError:
            param_dir = os.path.join(self.base_dir, dir_name, param_file)

        with open(param_dir,"r") as fairu:
            param = fairu.read()
        result = yaml.safe_load(param)
        node_name = self.get_name()
        for namespace in result.keys():
            if namespace == "/**":
                nodes = result[namespace]
                if node_name in nodes.keys():
                    tree_dict = nodes[node_name]["ros__parameters"]
                    ancestor_list = []
                    self.process_tree_dict(tree_dict, ancestor_list)

    def get_parameter_value(self, name:str):
        """returns value of parameter in yaml file"""
        if name in self.param_value_dict.keys():
            return self.param_value_dict[name] # Python
        else:
            return self.get_parameter(name).value # ROS2 Python 
        
    @staticmethod    
    def string_list_to_string(str_list:list[str],separator:str)->str:
        """convert list of strings to str+separator+str+...+str"""
        output = ""
        is_first = True
        for s in str_list:
            if is_first:
                output = output + s
            else:
                output = output + separator
        return output
    
    def process_tree_dict(self, tree_dict:dict , ancestor_list:list)->None:
        """ assign data type based on a fraction of types from dir(rclpy.parameter.Parameter.Type).
            basically change python data type to ROS2 python data type                                 """
        param_names = tree_dict.keys()
        for param_name in param_names:
            ancestor_list_temp = ancestor_list.copy() # avoid ancestor_list_temp = ancestor_list in python!
            ancestor_list_temp.append(param_name)
            if isinstance(tree_dict[param_name],dict):
                self.process_tree_dict(tree_dict[param_name],ancestor_list_temp)   # recursive loop until no child member
            elif isinstance(tree_dict[param_name],bool):                           # bool must be first to avoid bug, because issubclass(bool,int) == True
                declare_name = self.string_list_to_string(ancestor_list_temp,".")
                self.process_param_name_and_type(declare_name,Parameter.Type.BOOL) 
            elif isinstance(tree_dict[param_name],float):
                declare_name = self.string_list_to_string(ancestor_list_temp,".")
                self.process_param_name_and_type(declare_name,Parameter.Type.DOUBLE)
            elif isinstance(tree_dict[param_name],int):
                declare_name = self.string_list_to_string(ancestor_list_temp,".")
                self.process_param_name_and_type(declare_name,Parameter.Type.INTEGER)
            elif isinstance(tree_dict[param_name],str):
                declare_name = self.string_list_to_string(ancestor_list_temp,".")
                self.process_param_name_and_type(declare_name,Parameter.Type.STRING)
            elif isinstance(tree_dict[param_name],list):
                declare_name = self.string_list_to_string(ancestor_list_temp,".")
                if isinstance(tree_dict[param_name],bool):
                    self.process_param_name_and_type(declare_name,Parameter.Type.BOOL_ARRAY)
                elif isinstance(tree_dict[param_name],int):
                    self.process_param_name_and_type(declare_name,Parameter.Type.INTEGER_ARRAY)
                elif isinstance(tree_dict[param_name],str):
                    self.process_param_name_and_type(declare_name,Parameter.Type.STRING_ARRAY)
                elif isinstance(tree_dict[param_name],float):
                    self.process_param_name_and_type(declare_name,Parameter.Type.DOUBLE_ARRAY)      

    def process_param_name_and_type(self,name:str, type: Parameter.Type)->None:
        self.declare_parameter(name,0)
        self.param_value_dict[name] = self.get_parameter(name).value
        self.param_type_dict[name] = type
        self.param_callback_dict[name] = partial(self.param_value_dict.__setitem__, name)

if __name__ == "__main__":
    rclpy.init()
    config = ParamReader()
    config.load_parameters_from_file("robot_params.yaml", "CircularFormation")
    print(config.get_parameter_value("N"))
    config.destroy_node()
    rclpy.shutdown()
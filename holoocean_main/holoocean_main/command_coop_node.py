from holoocean_main.holoocean_interface import HolooceanInterface
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import numpy as np

from geometry_msgs.msg import Point
from holoocean_interfaces.msg import UAVCommand

import yaml
import os

class CommandExample(Node):

    def __init__(self):
        super().__init__('command_coop_node')

        self.declare_parameter('ros_params', '')
        ros_params_file_path = self.get_parameter('ros_params').get_parameter_value().string_value
        self.load_yaml_parameters(ros_params_file_path)

        # self._parameters: {'name': Parameter(value, type)}
        uav_command_topic = self._parameters['uav_command_topic'].value
        sv_command_topic =  self._parameters['sv_command_topic'].value

        self.declare_parameter('params_file', '')
        file_path = self.get_parameter('params_file').get_parameter_value().string_value
        interface = HolooceanInterface(file_path, init=False)

        self.time_warp = interface.get_time_warp()

        self.uav_command_publisher = self.create_publisher(UAVCommand, uav_command_topic, 10)
        self.sv_command_publisher = self.create_publisher(Point, sv_command_topic, 10)
        
        
        # self.use_random = self.get_parameter('random').get_parameter_value().bool_value
        self.use_random = False
        self.sequence_index = 0

        #Setup timer to continue publishing depth heading
        timer_publish_period = 0.5 / self.time_warp  # seconds
            
        self.timer_publish = self.create_timer(timer_publish_period, self.publish_callback)

    def publish_callback(self):
        uav_command = UAVCommand()
        uav_command.orientation = [.0, .0, .0]
        uav_command.altitude = 500
        self.uav_command_publisher.publish(uav_command)

        sv_command = Point()
        sv_command.x = .0
        sv_command.y = .0
        self.sv_command_publisher.publish(sv_command)


    def load_yaml_parameters(self, file_path):
        """Load parameters from YAML file"""
        try:
            # Expande caminhos com ~ para home directory
            expanded_path = os.path.expanduser(file_path)
            
            with open(expanded_path, 'r') as f:
                params_dict = yaml.safe_load(f) or {}
                
            # Verifica estrutura ROS 2 padrão (namespace -> ros__parameters)
            for namespace, params in params_dict.items():
                if 'command_coop_node' in namespace and 'ros__parameters' in params:
                    params_to_load = params['ros__parameters']
                else:
                    continue
                
                for param_name, param_value in params_to_load.items():
                    full_name = f'{param_name}' if namespace != '' else param_name
                    self.set_parameter(full_name, param_value)
                    
            self.get_logger().info(f'Parameters loaded from: {expanded_path}')
        except FileNotFoundError:
            self.get_logger().error(f'Configuration file not found: {file_path}')
        except yaml.YAMLError as e:
            self.get_logger().error(f'YAML parsing error in {file_path}: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error loading parameters from {file_path}: {str(e)}')

    def set_parameter(self, name, value):
        try:
            # Verifica se o parâmetro já foi declarado
            if not self.has_parameter(name):
                self.declare_parameter(name, value)
            
            # Converte o valor para o tipo correto do ROS 2
            param = Parameter(name, value=value)
            self.set_parameters([param])
            
        except Exception as e:
            self.get_logger().warn(f'Falha ao definir parâmetro {name}: {str(e)}')

    def log_parameters(self):
        params = self._parameters
        self.get_logger().info("Current parameters:")
        for name, param in params.items():
            self.get_logger().info(f"  {name}: {param.value} (type: {type(param.value).__name__})")

def main(args=None):
    rclpy.init(args=args)

    command_node = CommandExample()

    rclpy.spin(command_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    command_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
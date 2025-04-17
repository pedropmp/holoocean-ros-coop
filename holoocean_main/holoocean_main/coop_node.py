from holoocean_main.holoocean_interface import HolooceanInterface
from holoocean.vehicle_dynamics import *
from holoocean.dynamics import *

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.parameter import Parameter

from geometry_msgs.msg import Point
from holoocean_interfaces.msg import UAVCommand

import yaml
import os

class CoopNode(Node):
    
    def __init__(self):
        super().__init__('coop_node')
        
        self.declare_parameter('ros_params', '')
        ros_params_file_path = self.get_parameter('ros_params').get_parameter_value().string_value
        self.load_yaml_parameters(ros_params_file_path)

        # self._parameters: {'name': Parameter(value, type)}
        uav_command_topic = self._parameters['uav_command_topic'].value
        sv_command_topic =  self._parameters['sv_command_topic'].value

        ######## START HOLOOCEAN INTERFACE ###########
        self.declare_parameter('params_file', '')
        file_path = self.get_parameter('params_file').get_parameter_value().string_value

        self.interface = HolooceanInterface(file_path, node=self)

        self.create_publishers() #Holoocean Publishers
        self.timer = self.create_timer(self.interface.get_time_warp_period(), self.tick_callback)
        self.get_logger().info('Tick Started')

        self.sv_command = np.array(np.zeros(2),float)
        self.uav_command = np.array(np.zeros(4),float)
        
        ######## CUSTOM SUBSCRIBERS ############
        
        self.sv_sub = self.create_subscription(
            Point,
            uav_command_topic,
            self.sv_callback,
            10
        )

        self.uav_sub = self.create_subscription(
            UAVCommand,
            sv_command_topic,
            self.uav_callback,
            10
        )

        ######### CUSTOM SIMULATION INIT ########
        self.draw = False
        if "draw_arrow" in self.interface.scenario:
            self.draw = self.interface.scenario["draw_arrow"]
            
        self.use_rpm = False
   
    def tick_callback(self):
        #Tick the envionment and publish data as many times as requested
        commands = {'sv': self.sv_command, 'uav': self.uav_command}
        state = self.interface.tick(commands)
        self.interface.publish_sensor_data(state)

    def sv_callback(self, msg: Point):
        self.sv_command = np.array([msg.x, msg.y])

    def uav_callback(self, msg: UAVCommand):
        self.uav_command = np.array(msg.orientation+[msg.altitude])

    def create_publishers(self):
        for sensor in self.interface.sensors:
            sensor.publisher = self.create_publisher(sensor.message_type, sensor.name, 10)

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
        """Declara e define um parâmetro com tratamento de tipos"""
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
        """Registra todos os parâmetros carregados"""
        params = self._parameters
        self.get_logger().info("Parâmetros atuais:")
        for name, param in params.items():
            self.get_logger().info(f"  {name}: {param.value} (type: {type(param.value).__name__})")

def main(args=None):
    rclpy.init(args=args)
    node = CoopNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

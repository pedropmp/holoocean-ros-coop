from holoocean_main.holoocean_interface import HolooceanInterface
from holoocean.vehicle_dynamics import *
from holoocean.dynamics import *

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from geometry_msgs.msg import Point
from holoocean_interfaces.msg import UAVCommand


class CoopNode(Node):
    
    def __init__(self):
        super().__init__('coop_node')
        
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
            'sv_position_ref',
            self.sv_callback,
            10
        )

        self.uav_sub = self.create_subscription(
            UAVCommand,
            'uav_position_ref',
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

def main(args=None):
    rclpy.init(args=args)
    node = CoopNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

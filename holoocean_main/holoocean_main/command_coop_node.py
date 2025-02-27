from holoocean_main.holoocean_interface import HolooceanInterface
import rclpy
from rclpy.node import Node

import numpy as np

from geometry_msgs.msg import Point
from holoocean_interfaces.msg import UAVCommand


class CommandExample(Node):

    def __init__(self):
        super().__init__('command_coop_node')

        self.declare_parameter('params_file', '')
        
        file_path = self.get_parameter('params_file').get_parameter_value().string_value
        interface = HolooceanInterface(file_path, init=False)

        self.time_warp = interface.get_time_warp()

        self.uav_command_publisher = self.create_publisher(UAVCommand, 'uav_command', 10)
        self.sv_command_publisher = self.create_publisher(Point, 'sv_command', 10)
        
        
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
        sv_command.x = 0
        sv_command.y = 0
        self.sv_command_publisher.publish(sv_command)


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
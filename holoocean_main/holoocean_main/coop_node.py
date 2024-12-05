from holoocean_main.holoocean_interface import HolooceanInterface
from holoocean.vehicle_dynamics import *
from holoocean.dynamics import *

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, Float64


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

        self.accel = np.array(np.zeros(6),float)
        
        ######## CUSTOM SUBSCRIBERS ############
        
        # Heading in degrees (-180, 180) centered at NORTH?? 
        self.heading_sub = self.create_subscription(
            Float64,
            'heading',
            self.heading_callback,
            10
        )

        # Speed (surge) in x body frame (forward, m/s)
        self.speed_sub = self.create_subscription(
            Float64,
            'speed',
            self.speed_callback,
            10
        )

        ######### CUSTOM SIMULATION INIT ########
        self.draw = False
        if "draw_arrow" in self.interface.scenario:
            self.draw = self.interface.scenario["draw_arrow"]
            
        self.use_rpm = False
   
    def tick_callback(self):
        #Tick the envionment and publish data as many times as requested
        state = self.interface.tick(self.accel)
        self.interface.publish_sensor_data(state)

    def heading_callback(self,msg):
        pass
    def speed_callback(self, msg):
        pass

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

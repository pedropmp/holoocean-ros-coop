import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, Quaternion

from transforms3d.euler import euler2quat 

from pathlib import Path
import yaml

class RotationIMU(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu_rotation', 10)
        self.declare_parameter('params_file', '')
        file_path = self.get_parameter('params_file').get_parameter_value().string_value
        scenario_path = Path(file_path)
        agents_names = self.get_agents_names(scenario_path=scenario_path)
        self.create_subscribers(agents_names)
        self.q = euler2quat(0, 0, 0)
        # self.publisher_ = self.create_publisher(Imu, 'IMURotation', 10)
        # self.subscription = self.create_subscription(
        #     Imu,
        #     'topic',
        #     self.listener_callback,
        #     10)
        # timer_period = 1/300  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def timer_callback(self):
        msg = Imu()
        msg.orientation = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def parse_scenario_yaml(self, scenario_path):
        with open(scenario_path, 'r') as file:
            yaml_content = yaml.safe_load(file)
        
        holoocean_scenario_yaml = self.find_holoocean_scenario(yaml_content)
        
        if holoocean_scenario_yaml is None:
            raise KeyError("Could not find 'holoocean_scenario' in the YAML file.")
        
        # # Convert the 'holoocean_scenario' part to JSON
        # scenario = json.dumps(holoocean_scenario_yaml, indent=4)
        # print(holoocean_scenario_yaml)

        return holoocean_scenario_yaml
    
    def find_holoocean_scenario(self, yaml_content):
        """Recursively search for 'holoocean_scenario' in the YAML content."""
        if isinstance(yaml_content, dict):
            for key, value in yaml_content.items():
                if key == "holoocean_scenario":
                    return value
                else:
                    result = self.find_holoocean_scenario(value)
                    if result is not None:
                        return result
        elif isinstance(yaml_content, list):
            for item in yaml_content:
                result = self.find_holoocean_scenario(item)
                if result is not None:
                    return result
        return None

    def get_agents_names(self, scenario_path) -> list:
        scenario = self.parse_scenario_yaml(scenario_path)
        agents = scenario['agents']
        agents_names = [agent['agent_name'] for agent in agents]
        return agents_names

    def create_subscribers(self, agents_names):
        for agent_name in agents_names:
            topic = f"/holoocean/{agent_name}/IMUSensor"
            self.create_subscription(
                Imu,
                topic,
                self.imu_callback,
                10)
            
            topic = f"/holoocean/{agent_name}/RotationSensor"
            self.create_subscription(
                Vector3Stamped,
                topic,
                self.rotation_callback,
                10)
            
    def imu_callback(self, msg:Imu):
        # TODO: merge quaternion value stored with holoocean sensor msg
        msg.orientation._w = self.q[0]
        msg.orientation._x = self.q[1]
        msg.orientation._y = self.q[2]
        msg.orientation._z = self.q[3]
        self.publisher_.publish(msg)

    def rotation_callback(self, msg:Vector3Stamped):
        # TODO: read euler angles, transform to quarternion and store it in 
        # class atribute
        self.q = euler2quat(msg.vector.x, msg.vector.y, msg.vector.z)



def main(args=None):
    rclpy.init(args=args)

    imu_publisher = RotationIMU()
    rclpy.spin(imu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
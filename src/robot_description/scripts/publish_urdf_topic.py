#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import xacro
import os

class URDFPublisher(Node):
    def __init__(self):
        super().__init__('publish_urdf_topic')
        self.publisher = self.create_publisher(String, '/robot_description', 1)

        pkg_path = os.path.join(os.getenv('HOME'), 'ros2_model/src/robot_description/urdf/lego_robot.urdf.xacro')
        doc = xacro.process_file(pkg_path)
        msg = String()
        msg.data = doc.toxml()
        self.publisher.publish(msg)
        self.get_logger().info('Published robot_description to /robot_description')

def main(args=None):
    rclpy.init(args=args)
    node = URDFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
import threading
import struct
import time

from hokuyo.driver import hokuyo
import serial
from hokuyo.tools import serial_port

from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time
import numpy as np 

import math
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def to_float32(value: float) -> float:
    packed = struct.pack("f", value)
    unpacked = struct.unpack("f", packed)[0]
    return unpacked

class Lidar (Node):
    def __init__(self):
        super().__init__("lidar_publisher")

        self.laser_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.get_logger().info("Запущен лидар!")


        self.uart_port = '/dev/ttyACM0'
        self.uart_speed = 19200

        self.laser_ranges_size = 687
        self.distances = np.zeros((self.laser_ranges_size,))
        self.angles = np.zeros((self.laser_ranges_size,))

        self.laser_thread = threading.Thread(target=self.laser_pusk)
        self.laser_thread.start()
   
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        transformation = [0.0, 0.0, 0.18]
        self.make_transforms(transformation)

    def laser_pusk (self):

        try:
            laser_serial = serial.Serial(port=self.uart_port, baudrate=self.uart_speed, timeout= 1000)
            port = serial_port.SerialPort(laser_serial)
            self.laser = hokuyo.Hokuyo(port)
            self.laser.set_scip2()

            self.laser.laser_on()

            self.scan()

        except Exception as e:
            self.get_logger().error(f"Ошибка лидара: {e}")
        finally:
            self.laser.laser_off()
            self.laser.terminate()

    def scan (self):
        try:
            while rclpy.ok():
                data = self.laser.get_single_scan()

                if data == b'':
                    raise RuntimeError("socket connection broken")
                
                i = 0 
                for a, d in data.items():
                    d = d / 1000
                    a = a * math.pi/180
                    self.distances[i]= d
                    self.angles[i] = a
                    i+=1
                self.laser_publish(self.distances, self.angles)
        finally:
            self.laser_off()

    def laser_publish (self, d, a):

        lidar_msg = LaserScan()
        lidar_msg.header.stamp = self.get_clock().now().to_msg()
        lidar_msg.header.frame_id = "lidar"
        lidar_msg.angle_min = to_float32(-math.pi/6)
        lidar_msg.angle_max = to_float32(math.radians(210))

        # angle_increment_deg = (a[-1] - a[0]) / (len(a))
        lidar_msg.angle_increment = to_float32(math.radians((0.35)))

        lidar_msg.time_increment = to_float32(0.0)
        lidar_msg.scan_time = to_float32(0.1)
        lidar_msg.range_min = to_float32(min(d))
        lidar_msg.range_max = to_float32(max(d))
        lidar_msg.ranges = [to_float32(x) for x in d]

        self.laser_pub.publish(lidar_msg)

        self.distances = np.zeros((self.laser_ranges_size,))
        self.angles = np.zeros((self.laser_ranges_size,))

    def make_transforms(self, transformation):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'lidar_link'
        t.child_frame_id = 'lidar'

        t.transform.translation.x = float(transformation[0])
        t.transform.translation.y = float(transformation[1])
        t.transform.translation.z = float(transformation[2])
        t.transform.rotation.x = float(0)
        t.transform.rotation.y = float(0)
        t.transform.rotation.z = float(-np.sqrt(2)/2)
        t.transform.rotation.w = float(np.sqrt(2)/2)

        self.tf_static_broadcaster.sendTransform(t)

    def laser_off(self):
        try:
            if hasattr(self, 'laser'):
                self.laser.laser_off()
                self.laser.terminate()
                self.get_logger().info("Лазер выключен")
        except:
            pass


    def destroy_node(self):
        self.get_logger().info("Выключаем лазер...")
        self.is_running = False
        self.laser_off()
        time.sleep(0.2)
        super().destroy_node()

def main(args=None):

    rclpy.init(args=args)
    node = Lidar()
    
    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
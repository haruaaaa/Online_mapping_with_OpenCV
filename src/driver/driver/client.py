import time
import threading
import socket
import struct
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_geometry_msgs
from tf2_ros import TransformBroadcaster, TransformStamped

class Moving(Node):
    
    def __init__(self):
        super().__init__("moving")
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.speeds, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)


        self.Host = '10.42.0.35' 
        self.Port = 8089

        self.B = 0.159 
        self.r = 0.042
        self.T = 0.1

        self.linear = 0.0
        self.angular = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_connected = False
        
        try:
            self.socket.connect((self.Host, self.Port))
            self.socket_connected = True
            self.get_logger().info('Connected to server')

            self.send_thread = threading.Thread(target=self.send_data)
            self.rec_thread = threading.Thread(target=self.receive_data)
            self.send_thread.daemon = True  
            self.rec_thread.daemon = True
            self.send_thread.start()
            self.rec_thread.start()

            self.receive_timer = self.create_timer(self.T, self.publish_tf) 
                    
       
        except ConnectionRefusedError:
            self.get_logger().info(f'No connection')
            self.socket_connected = False
            
    
    def speeds(self, msg):

        self.linear = msg.linear.x
        self.angular = msg.angular.z

                
    def receive_data(self):
        while self.socket_connected:

            chunk = self.socket.recv(12)
            if chunk == b'':
                raise RuntimeError("socket connection broken")


            self.x, self.y, self.theta = struct.unpack(">fff", chunk)

            msg_odom = Odometry()
            msg_odom.header.stamp = self.get_clock().now().to_msg()
            msg_odom.header.frame_id = 'odom'
            msg_odom.pose.pose.position.x = self.x
            msg_odom.pose.pose.position.y = self.y
            msg_odom.pose.pose.position.z = 0.0

            msg_odom.pose.pose.orientation.x = 0.0
            msg_odom.pose.pose.orientation.y = 0.0
            msg_odom.pose.pose.orientation.z = math.sin(self.theta / 2)
            msg_odom.pose.pose.orientation.w = math.cos(self.theta / 2)

            
            msg_odom.child_frame_id = 'base_footprint'
            self.odom_pub.publish(msg_odom)
            time.sleep(self.T)

                
    # отправка данных на сервер
    def send_data(self):

        while self.socket_connected:
            
            send_check = self.socket.send(struct.pack(">ff", self.linear, self.angular))
            if send_check == 0:
                raise RuntimeError("socket connection broken")
        
            time.sleep(self.T)


    
    def publish_tf(self):

        point_st = tf2_geometry_msgs.PointStamped()
        point_st.header.stamp = self.get_clock().now().to_msg()
        point_st.header.frame_id = 'base_footprint'
        point_st.point.x = float(0.0)
        point_st.point.y = float(0.0)
        point_st.point.z = float(0.0)
        
        transform = TransformStamped()
        transform.header.stamp = point_st.header.stamp
        transform.header.frame_id = 'odom'      
        transform.child_frame_id = 'base_footprint'  
        
       
        transform.transform.translation.x = self.x 
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
       
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.theta / 2)
        transform.transform.rotation.w = math.cos(self.theta / 2)
        
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = Moving()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

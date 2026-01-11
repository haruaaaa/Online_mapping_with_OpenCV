import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import cv2
import math
from scipy.spatial import KDTree
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('explorer_cv')
        self.robot_x = 0
        self.robot_y = 0
        self.last_robot_x = 0 
        self.last_robot_y = 0 
        self.best_distance_to_goal = float('inf')

        self.robot_x_ = 0
        self.robot_y_ = 0

        self.frontier = None

        self.blacklisted_frontiers = set()
        
        self.create_subscription(OccupancyGrid, '/map', self.from_map_to_pic,10)
        
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.tf_buffer = Buffer() 
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
        self.create_subscription(Pose, '/marker', self.marker_callback, 10)
        self.standart_orientation = None
        self.marker_goal = False
        self.start_marker = None
        self.create_timer(1.0, self.check_marker_goal_status)
        

        self.goal_active = False
        self.goal_start_time = None
        self.create_timer(1.0, self.check_goal_status)
        self.create_timer(0.05, self.get_robot_pose_in_map)
        
        self.map_data = None
        self.map_info = None
        self.image = None
        self.binary_image = None

        self.current_goal = None
        self.contours = None
        self.contur_not_tuple = []
        self.point_not_found = True

        self.get_logger().info("Frontier Explorer initialized!!!")
    

    def get_robot_pose_in_map(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(seconds=0)) 
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info(f"TF transform failed: {e}")
        finally:
            if self.map_info != None:
                self.robot_x_ = int((self.robot_x - self.map_info.origin.position.x) / self.map_info.resolution)
                self.robot_y_ = int((self.robot_y - self.map_info.origin.position.y) / self.map_info.resolution)
        

    def from_map_to_pic(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.image = np.zeros((msg.info.height, msg.info.width), dtype=np.uint8)
        self.image[self.map_data == -1] = 0
        self.image[self.map_data >= 0] = 255

        self.morph()

    def morph(self):

        kernel = np.ones((4, 4), np.uint8)
        self.image = cv2.morphologyEx(self.image, cv2.MORPH_OPEN, kernel)
        self.image = cv2.morphologyEx(self.image, cv2.MORPH_CLOSE, kernel)
        _, self.binary_image = cv2.threshold(self.image, 0, 255, cv2.THRESH_BINARY)
        self.contours, hierarchy = cv2.findContours(self.binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.contur_not_tuple = [] 

        if self.contours is None:
            return
        
        if self.goal_active:
            return
        
        for contur in self.contours:
            for point in contur:
                x, y = point[0]
                if abs(x - self.robot_x_)>=4 and abs(y-self.robot_y_)>=4:
                    self.contur_not_tuple.append([x, y])
        
        self.contur_not_tuple = np.array(self.contur_not_tuple)

        self.filtering()


    def filtering(self):
        self.point_not_found = True

        if not np.any(self.contur_not_tuple):
            return
           
        while self.point_not_found:

            if self.contur_not_tuple.size == 0: 
                self.get_logger().warn("Все фронтиры исчерпаны или занесены в ЧС!")
                self.point_not_found = False
                return
            
            kd_tree = KDTree(self.contur_not_tuple)
            distance, nearest_idx = kd_tree.query([self.robot_x_, self.robot_y_])

            nearest_point_x = self.contur_not_tuple[nearest_idx][0] * self.map_info.resolution + self.map_info.origin.position.x
            nearest_point_y = self.contur_not_tuple[nearest_idx][1] * self.map_info.resolution + self.map_info.origin.position.y
            nearest_point = (nearest_point_x, nearest_point_y)

            if self.check_walls(self.contur_not_tuple[nearest_idx]):
                if nearest_point not in self.blacklisted_frontiers and nearest_point_x>=0:
                    
                    self.frontier = np.array([nearest_point_x, nearest_point_y])
                    self.circle_filter()
                else:
                    self.blacklisted_frontiers.add(nearest_point)
                    self.contur_not_tuple = np.delete(self.contur_not_tuple, nearest_idx, axis=0)
            else:
                self.blacklisted_frontiers.add(nearest_point)
                self.contur_not_tuple = np.delete(self.contur_not_tuple, nearest_idx, axis=0)
                    

    def check_walls(self, near_point):
        # Определяем границы окна
        px, py = near_point
        h, w = self.map_data.shape
        half_win = 0.14
        # можно изменить под параметры помещения
        half_win_pixels = int(round(half_win / self.map_info.resolution))
        
        if half_win_pixels == 0: # Защита от деления на 0 или слишком низкого resolution
            half_win_pixels = 1

        x_min = max(0, px - half_win_pixels)
        x_max = min(w, px + half_win_pixels + 1)
        y_min = max(0, py - half_win_pixels)
        y_max = min(h, py + half_win_pixels + 1)
        
        # Извлекаем данные карты в окне
        window = self.map_data[y_min:y_max, x_min:x_max]
        
        num_of_walls = window[window > 1].size

        if num_of_walls >= 10: 
            return False
        return True
    


    def circle_filter(self):
        radius = 0.13
        v = np.array([self.frontier[0] - self.robot_x, self.frontier[1] - self.robot_y])
        ort = v / np.linalg.norm(v, 2)
        center = self.frontier + (ort * radius)
        h, w = self.image.shape
    

        center_x = int((center[0] - self.map_info.origin.position.x) / self.map_info.resolution)
        center_y = int((center[1] - self.map_info.origin.position.y) / self.map_info.resolution)
        
        radius_in_map = int(radius / self.map_info.resolution)
        
        if radius_in_map == 0:
            radius_in_map = 1
        
        
        x_min = max(0, center_x - radius_in_map)
        x_max = min(w, center_x + radius_in_map + 1)
        y_min = max(0, center_y - radius_in_map)
        y_max = min(h, center_y + radius_in_map + 1)

        #создаём квадрат
        window = self.image[y_min:y_max, x_min:x_max]
        setchik = window[window == 0].size

        # self.get_logger().info(f"Заполненность: {setchik}/{window.size} = {setchik/window.size:.2f}")

        if setchik/window.size>=0.15:
            self.point_not_found = False
            
            self.standart_orientation = np.array([0.0, 0.0, 0.0, 1.0])
            
            self.publish_goal()
        else:
            self.blacklisted_frontiers.add(tuple(self.frontier))
            self.frontier = None
            
    def publish_goal(self): 
        if self.frontier is None:
            return 
               
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = float(self.frontier[0])
        goal_msg.pose.position.y = float(self.frontier[1])
        goal_msg.pose.position.z = 0.0
        
        goal_msg.pose.orientation.x = float(self.standart_orientation[0])
        goal_msg.pose.orientation.y = float(self.standart_orientation[1])
        goal_msg.pose.orientation.z = float(self.standart_orientation[2])
        goal_msg.pose.orientation.w = float(self.standart_orientation[3])
        
        self.goal_pub.publish(goal_msg)
        
        self.current_goal = (self.frontier[0], self.frontier[1])
        self.goal_active = True
        self.goal_start_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        self.get_logger().info(f"Фронтир в данный момент: ({self.frontier[0]:.2f}, {self.frontier[1]:.2f})")
        self.last_robot_x = self.robot_x
        self.last_robot_y = self.robot_y


    def check_goal_status(self):

        if not self.goal_active or self.current_goal is None or self.marker_goal:
            return
    
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        time_go = current_time - self.goal_start_time
        distance_to_goal = math.sqrt((self.robot_x - self.current_goal[0])**2 + (self.robot_y - self.current_goal[1])**2)
        distance_moved = math.sqrt((self.robot_x - self.last_robot_x)**2 + (self.robot_y - self.last_robot_y)**2)

        abort_goal = False
        abort_reason = ""
        if distance_to_goal < 0.15:
            self.get_logger().info(f"Цель достигнута!")
            self.reset_goal()
            return

        elif time_go > 30.0:
            abort_goal = True
            abort_reason = "превышено время достижения"
        
        elif time_go > 10.0 and distance_moved < 0.01:
            abort_goal = True
            abort_reason = "стоит на месте"

        elif distance_to_goal > self.best_distance_to_goal + 0.1:
            abort_goal = True
            abort_reason = "робот уезжает от цели"

        if abort_goal:
            self.get_logger().warn(f"Аборт цели {self.current_goal}: {abort_reason}")
            self.blacklisted_frontiers.add(self.current_goal)
            self.reset_goal()
            return
        
        if distance_to_goal < self.best_distance_to_goal:
            self.best_distance_to_goal = distance_to_goal
        
        self.last_robot_x = self.robot_x
        self.last_robot_y = self.robot_y
        

    def reset_goal(self):
        self.current_goal = None
        self.goal_active = False
        self.goal_start_time = None
        self.frontier = None
        self.marker_goal = False
        
        self.best_distance_to_goal = float('inf') 
        self.get_logger().info("Меняем")
        self.filtering()


    def marker_callback(self, msg):
        self.current_goal = None
        self.goal_active = False
        self.goal_start_time = None
        self.best_distance_to_goal = float('inf') 
        self.marker_goal = True
        marker_x = msg.position.x
        marker_y = msg.position.y 
        self.frontier = np.array([marker_x, marker_y])
        marker_orintation_x = msg.orientation.x 
        marker_orintation_y = msg.orientation.y
        marker_orintation_z = msg.orientation.z
        marker_orintation_w = msg.orientation.w
        self.standart_orientation = np.array([marker_orintation_x, marker_orintation_y, marker_orintation_z, marker_orintation_w])
        self.publish_goal()

    def check_marker_goal_status(self):
        if not self.marker_goal:
            return 
        distance_to_goal = np.sqrt((self.robot_x - self.current_goal[0])**2 + (self.robot_y - self.current_goal[1])**2)
        if distance_to_goal < 0.3:
            now = self.get_clock().now().seconds_nanoseconds()[0]
            self.get_logger().info(f"Маркер есть!")
            if self.start_marker is None:
                self.start_marker = self.get_clock().now().seconds_nanoseconds()[0]
            elif (now - self.start_marker)>5:
                self.get_logger().info(f"Едем дальше!")
                self.start_marker = None
                self.reset_goal()
                return
            else:
                return
###

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
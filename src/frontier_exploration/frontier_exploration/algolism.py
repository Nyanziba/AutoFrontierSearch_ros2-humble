# I will write a Python program that demonstrates how to analyze map data to detect frontiers and set a goal towards a detected frontier.
# This program is a basic implementation and should be adapted for specific requirements and the actual robot hardware.

# Import necessary ROS2 libraries
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatusArray
import numpy as np
import math
import random
from rclpy.time import Duration

class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')
        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.map_data = None
        self.map_array = None
        self.map_metadata = None
        self.goal_status_subscriber = self.create_subscription(
            GoalStatusArray,
            '/follow_path/_action/status',  # Replace with the actual topic name
            self.goal_status_callback,
            10)
        self.goal_reached = None
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.robot_x = 0.0
        
        self.robot_y = 0.0
        self.map_info = None
        self.timer = self.create_timer(5, self.timer_callback)
        
    

    def goal_status_callback(self, msg):
        if msg.status_list:
            # 最新のゴールステータスを取得
            current_status = msg.status_list[-1].status
            print(current_status)
            # ゴールに達していない場合、Goal_reached を False に設定
            if current_status == 3:
                self.goal_reached = True
                print("Goal reached.")
            elif current_status == 2:
                self.goal_reached = False
                print("active.")
            elif current_status == 4:
                self.goal_reached = True
                print("goal abort.")
            else:
                self.goal_reached = True

            # 現在のゴールの状態をログに記録
            self.get_logger().info(f"Goal reached: {self.goal_reached}")
    
    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_metadata = msg.info
        self.map_info = msg.info
        
        
                
    def timer_callback(self):
        if self.map_array.any():
            
            if self.goal_reached==True:
                self.get_logger().info(f"Map info received: Resolution: {self.map_info.resolution}, Width: {self.map_info.width}, Height: {self.map_info.height}")
                print("Goal reached, setting nearby frontier.")
                frontiers = self.detect_frontiers(map_info=self.map_info)
                goal = self.select_goal(frontiers, self.map_array)
                self.publish_goal(goal)

            else:
                print("Goal not reached, waiting.")
                pass

                
    

    
    def odom_callback(self, msg):
        # Update the robot's current position based on the odometry data
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
            
    def is_valid_cell(self, x, y, map_info):
        # セルがマップの有効範囲内にあるか確認
        # 例えば、マップの境界をチェック
        return 0 <= x < map_info.width and 0 <= y < map_info.height
    

    def is_near_wall(self, x, y, map_data, threshold):
        # この関数は、指定された座標が壁に近いかどうかを判定します。
        # x, yはフロンティアの座標、map_dataはマップのデータ、
        # thresholdは壁と判定するための距離のしきい値です。

        # マップの周囲のセルを調べる
        for i in range(-threshold, threshold+1):
            for j in range(-threshold, threshold+1):
                # マップデータ内での位置を計算
                check_x = x + i
                check_y = y + j

                # マップの範囲内かどうか確認
                if 0 <= check_x < map_data.shape[0] and 0 <= check_y < map_data.shape[1]:
                    if map_data[check_x, check_y] == 100:  # 壁を示す値
                        return True
        return False
    
    def detect_frontiers(self, map_info ):
        # Detecting frontiers (edges of explored and unexplored areas)
        # For simplicity, this is a very basic implementation and should be enhanced for real-world applications.
        frontiers = []
        if self.goal_reached==True:
            self.get_logger().info(f"Map info received: Resolution: {map_info.resolution}, Width: {map_info.width}, Height: {map_info.height}")
            
            for y in range(map_info.height):
                for x in range(map_info.width):
                    # マップデータの範囲内にあるセルのみを検討
                
                    
                        if self.map_array[y, x] == -1:  # -1 indicates unknown area in the map
                            neighbors = self.map_array[y-1:y+2, x-1:x+2]
                            if 0 in neighbors:  # 0 indicates free space in the map

                                frontiers.append((x, y))
            return frontiers
        else:
            return None

    

    def select_goal(self, frontiers, map_data):
        valid_frontiers = []
        for x, y in frontiers:
            if not self.is_near_wall(x, y, map_data, threshold=3):
                valid_frontiers.append((x, y))

        if valid_frontiers:
            # ランダムなフロンティアを選択
            random_frontier = random.choice(valid_frontiers)
            goal_x, goal_y = random_frontier
            # 実世界の座標に変換
            real_x = goal_x * self.map_metadata.resolution + self.map_metadata.origin.position.x
            real_y = goal_y * self.map_metadata.resolution + self.map_metadata.origin.position.y
            return real_x, real_y
        else:
            return None, None

    def publish_goal(self, goal):
        # Publishing the goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"  # Assuming the map frame
        goal_msg.pose.position = Point(x=goal[0] * 10*self.map_metadata.resolution,
                                       y=goal[1] * 10*self.map_metadata.resolution,
                                       z=0.0)  # Assuming flat terrain
        # Orientation is not set for simplicity
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Goal: {goal_msg.pose.position.x}, {goal_msg.pose.position.y}")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

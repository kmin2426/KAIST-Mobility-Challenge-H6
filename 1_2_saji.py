import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Accel, PoseStamped
import pandas as pd
import math

class TrafficManager(Node):
    def __init__(self):
        super().__init__('traffic_manager')
        
        # Danger Zone 로드
        self.dz1 = pd.read_csv('extracted_dz_51_46_1.csv')
        self.dz2 = pd.read_csv('extracted_dz_60_52_1.csv')
        
        self.pose1 = None
        self.pose2 = None
        self.priority = None 

        self.create_subscription(PoseStamped, '/CAV_01', self.pose1_cb, 10)
        self.create_subscription(PoseStamped, '/CAV_02', self.pose2_cb, 10)
        self.create_subscription(Accel, '/CAV_01_accel_raw', self.accel1_cb, 10)
        self.create_subscription(Accel, '/CAV_02_accel_raw', self.accel2_cb, 10)
        
        self.pub1 = self.create_publisher(Accel, '/CAV_01_accel', 10)
        self.pub2 = self.create_publisher(Accel, '/CAV_02_accel', 10)

        self.get_logger().info("🛡️ 관제탑: 모니터링 강도를 높였습니다.")

    def pose1_cb(self, msg): self.pose1 = msg.pose.position
    def pose2_cb(self, msg): self.pose2 = msg.pose.position

    def is_in_dz(self, pos, dz_df):
        if pos is None: return False
        # 임계값을 0.5m로 확장하여 더 일찍 감지
        dists_sq = (dz_df['X'] - pos.x)**2 + (dz_df['Y'] - pos.y)**2
        return dists_sq.min() < 0.5**2

    def accel1_cb(self, msg):
        if self.pose1 is None or self.pose2 is None:
            self.pub1.publish(msg); return
        
        in_dz1 = self.is_in_dz(self.pose1, self.dz1)
        in_dz2 = self.is_in_dz(self.pose2, self.dz2)
        
        # [핵심 로직] 2번이 이미 Danger Zone 근처에 있고, 1번보다 먼저 통행권을 가졌다면 정지
        if in_dz1 and in_dz2 and self.priority == 2:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif in_dz1 and self.priority is None:
            self.priority = 1
            self.get_logger().info("🔵 CAV_01 통행권 획득")

        # 두 차 모두 DZ를 완전히 벗어나면 우선순위 초기화 (안전거리 확보)
        if not in_dz1 and not in_dz2:
            self.priority = None

        self.pub1.publish(msg)

    def accel2_cb(self, msg):
        if self.pose1 is None or self.pose2 is None:
            self.pub2.publish(msg); return
        
        in_dz1 = self.is_in_dz(self.pose1, self.dz1)
        in_dz2 = self.is_in_dz(self.pose2, self.dz2)
        
        if in_dz2 and in_dz1 and self.priority == 1:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif in_dz2 and self.priority is None:
            self.priority = 2
            self.get_logger().info("🔴 CAV_02 통행권 획득")

        self.pub2.publish(msg)

def main():
    rclpy.init(); node = TrafficManager(); rclpy.spin(node); rclpy.shutdown()

if __name__ == '__main__':
    main()
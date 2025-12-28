import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Accel, PoseStamped
from rclpy.qos import qos_profile_sensor_data
import json
import math
import time
import os


# 파라미터 설정 
PATH_FILENAME = "new_path.json"
LOOKAHEAD_DIST = 0.3   
MAX_SPEED = 1.2        
MIN_SPEED = 0.3        
STEER_GAIN = 1.0       
STEER_FILTER_ALPHA = 0.7  
PID_KP = 0.8   
PID_KI = 0.01  
PID_KD = 0.0   
CTE_GAIN = 1.5 

class PurePursuitID1(Node):
    def __init__(self):
        super().__init__('pure_pursuit_id1')

        self.pose_sub = self.create_subscription(
            PoseStamped, '/CAV_01', self.pose_callback, qos_profile_sensor_data)
        self.control_pub = self.create_publisher(Accel, '/CAV_01_accel', 10)

        self.path_x, self.path_y = [], []
        self.load_path()

        self.last_nearest_idx = 0 
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.is_pose_received = False
        
        self.prev_angular_vel = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_time = time.time()
        self.current_speed = 0.0
        self.speed_error_sum = 0.0 
        self.prev_speed_error = 0.0 

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"🚀 [ID 1] Pure Pursuit Mode ON (Path: {PATH_FILENAME})")

    def load_path(self):
        """new_path.json 형식에 맞게 로드 로직 수정"""
        script_dir = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(script_dir, PATH_FILENAME)
        
        try:
            if os.path.exists(path):
                with open(path, 'r') as f:
                    data = json.load(f)
                    # 리스트 내 딕셔너리 구조에서 x, y 추출
                    self.path_x = [p['x'] for p in data]
                    self.path_y = [p['y'] for p in data]
                self.get_logger().info(f"📂 {len(self.path_x)}개의 경로점 로드 완료")
            else:
                self.get_logger().error(f"❌ 파일을 찾을 수 없습니다: {path}")
        except Exception as e:
            self.get_logger().error(f"❌ 로드 중 오류: {e}")

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        q = msg.pose.orientation
        # Quaternion to Yaw 변환
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        curr_time = time.time()
        dt = curr_time - self.prev_time
        if dt > 0:
            dist = math.hypot(self.current_x - self.prev_x, self.current_y - self.prev_y)
            self.current_speed = dist / dt
        
        self.prev_x = self.current_x
        self.prev_y = self.current_y
        self.prev_time = curr_time
        self.is_pose_received = True

    def get_nearest_index(self):
        # 뒤로 점프 방지를 위해 현재 인덱스 주변 탐색
        search_range = 200 
        start_idx = self.last_nearest_idx
        end_idx = min(start_idx + search_range, len(self.path_x))
        
        min_d = float('inf')
        nearest_idx = start_idx

        for i in range(start_idx, end_idx):
            d = (self.current_x - self.path_x[i])**2 + (self.current_y - self.path_y[i])**2
            if d < min_d:
                min_d = d
                nearest_idx = i
        
        self.last_nearest_idx = nearest_idx
        return nearest_idx

    def control_loop(self):
        if not self.is_pose_received or not self.path_x: return

        nearest_idx = self.get_nearest_index()

        # 1. 가변 Lookahead 적용
        current_lookahead = max(LOOKAHEAD_DIST, self.current_speed * 0.5)
        current_lookahead = min(current_lookahead, 1.0) 

        target_idx = nearest_idx
        while target_idx < len(self.path_x) - 1:
            if math.hypot(self.path_x[target_idx] - self.current_x, 
                          self.path_y[target_idx] - self.current_y) > current_lookahead:
                break
            target_idx += 1

        tx, ty = self.path_x[target_idx], self.path_y[target_idx]
        
        # 2. 로컬 좌표계 변환
        dx, dy = tx - self.current_x, ty - self.current_y
        local_x = math.cos(-self.current_yaw) * dx - math.sin(-self.current_yaw) * dy
        local_y = math.sin(-self.current_yaw) * dx + math.cos(-self.current_yaw) * dy

        # 3. Pure Pursuit & CTE 조향 결합
        curvature = 2.0 * local_y / (current_lookahead ** 2)
        
        near_dx = self.path_x[nearest_idx] - self.current_x
        near_dy = self.path_y[nearest_idx] - self.current_y
        cte_y = math.sin(-self.current_yaw) * near_dx + math.cos(-self.current_yaw) * near_dy

        final_steer_cmd = (curvature * STEER_GAIN) + (cte_y * CTE_GAIN)
        
        # 4. 속도 및 PID 제어
        steering_angle = math.atan(curvature * 0.3)
        target_speed = MAX_SPEED - (abs(steering_angle) * (MAX_SPEED - MIN_SPEED) * 0.8)
        target_speed = max(MIN_SPEED, min(MAX_SPEED, target_speed))

        dt = 0.05
        error = target_speed - self.current_speed 
        self.speed_error_sum = max(-1.0, min(1.0, self.speed_error_sum + error * dt))
        accel_cmd = (PID_KP * error) + (PID_KI * self.speed_error_sum)

        # 5. 최종 명령 발행 (필터링 적용)
        raw_angular_vel = (self.current_speed if self.current_speed > 0.1 else target_speed) * final_steer_cmd
        filtered_angular_vel = (STEER_FILTER_ALPHA * raw_angular_vel) + ((1.0 - STEER_FILTER_ALPHA) * self.prev_angular_vel)
        self.prev_angular_vel = filtered_angular_vel

        cmd_msg = Accel()
        cmd_msg.linear.x = float(accel_cmd)
        cmd_msg.angular.z = float(filtered_angular_vel)
        self.control_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitID1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
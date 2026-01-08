import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Accel, PoseStamped
from rclpy.qos import qos_profile_sensor_data
import json
import math
import time

# 파라미터 설정
LOOKAHEAD_DIST = 0.3   
MAX_SPEED = 1.2        
MIN_SPEED = 0.3        
STEER_GAIN = 1.0       
STEER_FILTER_ALPHA = 0.7 
PID_KP = 0.8   
PID_KI = 0.01  
PID_KD = 0.0   
CTE_GAIN = 1.5 

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

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
        
        # PID 변수
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_time = time.time()
        self.current_speed = 0.0
        self.speed_error_sum = 0.0 
        self.prev_speed_error = 0.0 

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Tuned Precision Mode ON (L={LOOKAHEAD_DIST}, G={STEER_GAIN}, CTE={CTE_GAIN})")

    def load_path(self):
        try:
            with open("path_shifted.json", 'r') as f:
                data = json.load(f)
                self.path_x = data['X']
                self.path_y = data['Y']
        except Exception as e: pass

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        q = msg.pose.orientation
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

    def check_reset_condition(self):
        if not self.path_x: return
        start_x = self.path_x[0]
        start_y = self.path_y[0]
        dist_to_start = math.hypot(self.current_x - start_x, self.current_y - start_y)

        if self.last_nearest_idx > 100 and dist_to_start < 0.5:
            self.get_logger().warn(">>> R key detected! Resetting... <<<")
            self.last_nearest_idx = 0
            self.prev_angular_vel = 0.0
            self.speed_error_sum = 0.0
            self.prev_speed_error = 0.0
            self.current_speed = 0.0

    def get_nearest_index(self):
        self.check_reset_condition()
        search_range = 200 
        start_idx = self.last_nearest_idx
        end_idx = min(start_idx + search_range, len(self.path_x))
        dx = [self.current_x - icx for icx in self.path_x[start_idx:end_idx]]
        dy = [self.current_y - icy for icy in self.path_y[start_idx:end_idx]]
        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
        if not d: return self.last_nearest_idx
        min_d = min(d)
        nearest_idx = start_idx + d.index(min_d)
        self.last_nearest_idx = nearest_idx
        return nearest_idx

    def control_loop(self):
        if not self.is_pose_received or not self.path_x: return

        nearest_idx = self.get_nearest_index()

        # 가변 Lookahead 적용

        current_lookahead = max(LOOKAHEAD_DIST, self.current_speed * 0.5)
        current_lookahead = min(current_lookahead, 1.0) # 최대 1m 제한

        target_idx = nearest_idx
        while target_idx < len(self.path_x) - 1:
            dx = self.path_x[target_idx] - self.current_x
            dy = self.path_y[target_idx] - self.current_y
            if math.hypot(dx, dy) > LOOKAHEAD_DIST: break
            target_idx += 1

        tx = self.path_x[target_idx]
        ty = self.path_y[target_idx]
        
        dx = tx - self.current_x
        dy = ty - self.current_y
        local_x = math.cos(-self.current_yaw) * dx - math.sin(-self.current_yaw) * dy
        local_y = math.sin(-self.current_yaw) * dx + math.cos(-self.current_yaw) * dy

        # 1. Pure Pursuit Curvature
        curvature = 2.0 * local_y / (LOOKAHEAD_DIST ** 2)
        
        # 2. Crosstrack Error (CTE) Calculation
        # 현재 차 위치에서 가장 가까운 경로점까지의 횡방향 거리 계산
        near_dx = self.path_x[nearest_idx] - self.current_x
        near_dy = self.path_y[nearest_idx] - self.current_y
        
        # Body Frame으로 변환된 횡방향 오차 (왼쪽 +, 오른쪽 -)
        cte_y = math.sin(-self.current_yaw) * near_dx + math.cos(-self.current_yaw) * near_dy

        # 3. Adaptive Speed
        steering_angle = math.atan(curvature * 0.3)
        target_speed = MAX_SPEED - (abs(steering_angle) * (MAX_SPEED - MIN_SPEED) * 0.8)
        target_speed = max(MIN_SPEED, min(MAX_SPEED, target_speed))

        # 4. PID Control
        dt = 0.05
        error = target_speed - self.current_speed 
        
        p_term = PID_KP * error
        self.speed_error_sum += error * dt
        if self.current_speed < 0.1: self.speed_error_sum = 0.0
        self.speed_error_sum = max(-1.0, min(1.0, self.speed_error_sum))
        i_term = PID_KI * self.speed_error_sum
        d_term = 0.0 
        
        accel_cmd = p_term + i_term + d_term
        if self.current_speed < 0.1 and accel_cmd < 0.0: accel_cmd = 0.0

        # 5. Steering Command mixing: (curvature * STEER_GAIN) + (cte_y * CTE_GAIN)
        
        pp_steer = curvature * STEER_GAIN
        cte_steer = cte_y * CTE_GAIN
        
        # 최종 조향 명령 (단위: curvature와 유사한 조향 비율)
        final_steer_cmd = pp_steer + cte_steer
        
        # 각속도로 변환 (v * k)
        raw_angular_vel = self.current_speed * final_steer_cmd
        
        # 저속 보호
        if self.current_speed < 0.1: 
             raw_angular_vel = target_speed * final_steer_cmd

        filtered_angular_vel = (STEER_FILTER_ALPHA * raw_angular_vel) + ((1.0 - STEER_FILTER_ALPHA) * self.prev_angular_vel)
        self.prev_angular_vel = filtered_angular_vel

        cmd_msg = Accel()
        cmd_msg.linear.x = float(accel_cmd)
        cmd_msg.angular.z = float(filtered_angular_vel)
        self.control_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
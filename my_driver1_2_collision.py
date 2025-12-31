import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import csv
import os
import math
import sys
from geometry_msgs.msg import Accel, PoseStamped
from std_msgs.msg import Int32

# ===============================================================
# [1] 설정 영역
# ===============================================================
target_vehicle_id = 1 
if len(sys.argv) > 1:
    try: target_vehicle_id = int(sys.argv[1])
    except: pass

DANGER_ZONE_CSV = 'rotary_1_2.csv' 

if target_vehicle_id == 1:
    MY_PATH_FILE = 'converted_path1_1.json'
    MY_TOPIC = '/CAV_01'
    OTHER_TOPIC = '/CAV_02'
    print(f"\n🔵 [차량 1] Priority Mode (Steering Boosted)")
elif target_vehicle_id == 2:
    MY_PATH_FILE = 'converted_path1_2.json'
    MY_TOPIC = '/CAV_02'
    OTHER_TOPIC = '/CAV_01'
    print(f"\n🔴 [차량 2] Priority Mode (Steering Boosted)")
else:
    MY_PATH_FILE = 'converted_path2.json'
    MY_TOPIC = '/CAV_01'
    OTHER_TOPIC = '/CAV_02'

# ===============================================================
# [2] 튜닝 파라미터 
# ===============================================================
SAFETY_DISTANCE = 0.5
ZONE_TOLERANCE = 0.2

TARGET_VELOCITY = 0.38
LOOK_AHEAD_DISTANCE = 0.23  

# PID 및 보정 게인
Kp = 4.0      
Ki = 0.05
Kd = 1.5      
K_cte = 5.0  

class PriorityRotaryDriver(Node):
    def __init__(self):
        super().__init__(f'priority_driver_{target_vehicle_id}')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(PoseStamped, MY_TOPIC, self.pose_callback, qos_profile)
        self.accel_publisher = self.create_publisher(Accel, f'{MY_TOPIC}_accel', 10)
        self.create_subscription(PoseStamped, OTHER_TOPIC, self.other_pose_callback, qos_profile)
        
        self.path_x = []
        self.path_y = []
        self.danger_zone_points = [] 
        
        self.load_my_path()
        self.load_danger_zone()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.is_pose_received = False
        self.other_car_x = None
        self.other_car_y = None
        
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.dt = 0.05 
        self.timer = self.create_timer(self.dt, self.drive_callback)
        self.log_counter = 0

    def load_my_path(self):
        if os.path.exists(MY_PATH_FILE):
            with open(MY_PATH_FILE, 'r') as f:
                data = json.load(f)
                self.path_x = data.get('X') or data.get('x') or []
                self.path_y = data.get('Y') or data.get('y') or []
        else: self.get_logger().error(f"❌ 경로 파일 없음")

    def load_danger_zone(self):
        if os.path.exists(DANGER_ZONE_CSV):
            with open(DANGER_ZONE_CSV, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    try:
                        px, py = float(row[0]), float(row[1])
                        if not (math.isnan(px) or math.isnan(py)):
                            self.danger_zone_points.append((px, py))
                    except: continue

    def pose_callback(self, msg):
        self.is_pose_received = True
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def other_pose_callback(self, msg):
        self.other_car_x = msg.pose.position.x
        self.other_car_y = msg.pose.position.y

    def is_in_zone(self, x, y):
        if not self.danger_zone_points: return False
        for (zx, zy) in self.danger_zone_points:
            if math.hypot(zx - x, zy - y) < ZONE_TOLERANCE: return True
        return False

    def drive_callback(self):
        if not self.is_pose_received or len(self.path_x) == 0: return

        # 1. 가장 가까운 경로점(CTE 계산용)
        min_dist = float('inf')
        current_idx = 0
        for i in range(len(self.path_x)):
            dist = math.hypot(self.path_x[i] - self.current_x, self.path_y[i] - self.current_y)
            if dist < min_dist:
                min_dist = dist
                current_idx = i

        # 2. 우선권 로직 (멈춤/출발 결정)
        final_velocity = TARGET_VELOCITY
        status_msg = "🟢 주행"

        if self.other_car_x is not None:
            dist_to_other = math.hypot(self.current_x - self.other_car_x, self.current_y - self.other_car_y)
            am_i_in_zone = self.is_in_zone(self.current_x, self.current_y)
            is_other_in_zone = self.is_in_zone(self.other_car_x, self.other_car_y)

            if am_i_in_zone:
                final_velocity = TARGET_VELOCITY # 내가 우선
            elif is_other_in_zone and (dist_to_other <= SAFETY_DISTANCE):
                final_velocity = 0.0 # 양보
                status_msg = "⛔ 양보 정지"

        # 3. Look Ahead Point 찾기
        target_idx = current_idx
        for i in range(current_idx, len(self.path_x)):
            if math.hypot(self.path_x[i] - self.current_x, self.path_y[i] - self.current_y) >= LOOK_AHEAD_DISTANCE:
                target_idx = i
                break
        
        tx = self.path_x[target_idx]
        ty = self.path_y[target_idx]

        # 4. 조향각 계산 (Steering Logic 강화됨)
        desired_yaw = math.atan2(ty - self.current_y, tx - self.current_x)
        yaw_err = desired_yaw - self.current_yaw
        
        # 각도 정규화 (-pi ~ pi)
        while yaw_err > math.pi: yaw_err -= 2 * math.pi
        while yaw_err < -math.pi: yaw_err += 2 * math.pi

        # PID 제어
        self.integral_error += yaw_err * self.dt
        self.integral_error = max(-1.0, min(1.0, self.integral_error)) # Windup 방지
        
        p_term = Kp * yaw_err
        i_term = Ki * self.integral_error
        d_term = Kd * (yaw_err - self.prev_error) / self.dt 
        
        # 💡 [핵심 수정] 거리 오차(CTE) 보정 강화
        # 경로가 내 진행방향 기준 왼쪽에 있는지 오른쪽에 있는지 판단하여 보정 방향 결정
        # 간단한 휴리스틱: yaw_err와 부호를 맞춤 (타겟이 왼쪽에 있으면 왼쪽으로 더 꺾어라)
        cte_correction = min_dist * K_cte  
        if yaw_err < 0: 
            cte_correction = -cte_correction 
        
        final_steering = p_term + i_term + d_term + cte_correction
        self.prev_error = yaw_err
        
        # 조향 제한
        final_steering = max(min(final_steering, 1.0), -1.0)
        
        cmd = Accel()
        cmd.linear.x = final_velocity
        cmd.angular.z = final_steering
        self.accel_publisher.publish(cmd)

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            print(f"[{current_idx}] {status_msg} | Err:{min_dist:.2f}m | Steer:{final_steering:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PriorityRotaryDriver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

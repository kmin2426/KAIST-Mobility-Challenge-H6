#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import os
import math
import sys
from geometry_msgs.msg import Accel, PoseStamped

# ===============================================================
# [1] 설정 로직 (실행 인자로 차량 구분)
# ===============================================================

target_vehicle_id = 1 

if len(sys.argv) > 1:
    try:
        target_vehicle_id = int(sys.argv[1])
    except ValueError:
        pass

if target_vehicle_id == 1:
    PATH_FILENAME = 'path1_1.json'   
    VEHICLE_TOPIC_NAME = '/CAV_01'           
    SPEED_LIMIT_TOPIC = '/CAV_01_target_speed' # 관제탑 명령 토픽
    print(f"\n🔵 [차량 1 제어 모드] 시작 (관제탑 연동)")

elif target_vehicle_id == 2:
    PATH_FILENAME = 'path1_2.json'   
    VEHICLE_TOPIC_NAME = '/CAV_02'           
    SPEED_LIMIT_TOPIC = '/CAV_02_target_speed' # 관제탑 명령 토픽
    print(f"\n🔴 [차량 2 제어 모드] 시작 (관제탑 연동)")

else:
    PATH_FILENAME = 'path1_1.json'
    VEHICLE_TOPIC_NAME = '/CAV_01'
    SPEED_LIMIT_TOPIC = '/CAV_01_target_speed'

# ===============================================================
# [2] PID 제어기
# ===============================================================
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.error_sum = 0.0
        self.last_error = 0.0

    def compute(self, error):
        p_term = self.kp * error
        self.error_sum += error * self.dt
        if self.error_sum > 5.0: self.error_sum = 5.0
        if self.error_sum < -5.0: self.error_sum = -5.0
        i_term = self.ki * self.error_sum
        d_term = self.kd * (error - self.last_error) / self.dt
        self.last_error = error
        return p_term + i_term + d_term

# ===============================================================
# [3] 메인 드라이버 노드 (관제탑 연동 고도화)
# ===============================================================
class PrecisionDriver(Node):
    def __init__(self):
        super().__init__(f'driver_vehicle_{target_vehicle_id}')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # [기존] 위치 정보 구독 및 가속도 명령 발행
        self.create_subscription(PoseStamped, VEHICLE_TOPIC_NAME, self.pose_callback, qos_profile)
        self.accel_publisher = self.create_publisher(Accel, f'{VEHICLE_TOPIC_NAME}_accel', 10)
        
        # [추가 1] 관제탑(inter_g.py)의 속도 제어 명령 구독
        self.target_speed = 0.48 # 초기값은 정상 속도
        self.create_subscription(Accel, SPEED_LIMIT_TOPIC, self.speed_limit_callback, 10)

        self.path_x = []
        self.path_y = []
        self.load_path_file()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.is_pose_received = False

        self.steering_pid = PIDController(kp=4.5, ki=0.05, kd=2.3, dt=0.05)
        self.timer = self.create_timer(0.05, self.drive_callback)

    # [추가 2] 관제탑 속도 명령을 업데이트하는 콜백 함수
    def speed_limit_callback(self, msg):
        # 관제탑에서 발행한 linear.x 값을 현재 목표 속도로 설정
        self.target_speed = msg.linear.x

    def load_path_file(self):
        if os.path.exists(PATH_FILENAME):
            with open(PATH_FILENAME, 'r') as f:
                data = json.load(f)
                self.path_x = data.get('X') or data.get('x') or []
                self.path_y = data.get('Y') or data.get('y') or []
                self.get_logger().info(f"✅ 경로 로드 완료 ({len(self.path_x)} points)")
        else:
            self.get_logger().error(f"❌ 경로 파일 없음: {PATH_FILENAME}")

    def pose_callback(self, msg):
        if not self.is_pose_received:
            self.get_logger().info(f"✨ 시뮬레이터 연결 성공! ({VEHICLE_TOPIC_NAME})")
            self.is_pose_received = True
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def drive_callback(self):
        if not self.is_pose_received or len(self.path_x) == 0:
            return

        # 1. 가장 가까운 점 찾기
        min_dist = float('inf')
        current_idx = 0
        for i in range(len(self.path_x)):
            dist = math.hypot(self.path_x[i] - self.current_x, self.path_y[i] - self.current_y)
            if dist < min_dist:
                min_dist = dist
                current_idx = i

        # 2. Look Ahead Point 계산
        LOOK_AHEAD = 0.23
        target_idx = current_idx
        for i in range(current_idx, len(self.path_x)):
            dist = math.hypot(self.path_x[i] - self.current_x, self.path_y[i] - self.current_y)
            if dist >= LOOK_AHEAD:
                target_idx = i
                break
        
        tx = self.path_x[target_idx]
        ty = self.path_y[target_idx]

        # 3. 조향 계산
        desired_yaw = math.atan2(ty - self.current_y, tx - self.current_x)
        yaw_err = desired_yaw - self.current_yaw
        while yaw_err > math.pi: yaw_err -= 2 * math.pi
        while yaw_err < -math.pi: yaw_err += 2 * math.pi

        steer_cmd = self.steering_pid.compute(yaw_err)
        cte_gain = 5.0
        cte = min_dist * cte_gain
        if yaw_err < 0: cte = -cte 
        
        final_steer = steer_cmd + cte
        final_steer = max(min(final_steer, 1.0), -1.0)
        
        # [추가 3] 최종 명령 발행 - 관제탑에서 받은 유동적인 속도 적용
        cmd = Accel()
        cmd.linear.x = self.target_speed # 기존 고정값 0.48에서 변경
        cmd.angular.z = final_steer
        self.accel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

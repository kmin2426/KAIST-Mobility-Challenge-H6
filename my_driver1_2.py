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

# 기본값
target_vehicle_id = 1 

# 실행할 때 뒤에 숫자를 붙였는지 확인 (예: python3 driver.py 2)
if len(sys.argv) > 1:
    try:
        target_vehicle_id = int(sys.argv[1])
    except ValueError:
        pass

if target_vehicle_id == 1:
    PATH_FILENAME = 'converted_path2.json'   # 1번 차량 경로
    VEHICLE_TOPIC_NAME = '/CAV_01'           # 1번 차량 토픽
    print(f"\n🔵 [차량 1 제어 모드] 시작")
    print(f"   - 경로 파일: {PATH_FILENAME}")
    print(f"   - 타겟 토픽: {VEHICLE_TOPIC_NAME}")

elif target_vehicle_id == 2:
    PATH_FILENAME = 'converted_path3.json'   # 2번 차량 경로
    VEHICLE_TOPIC_NAME = '/CAV_02'           # 2번 차량 토픽
    print(f"\n🔴 [차량 2 제어 모드] 시작")
    print(f"   - 경로 파일: {PATH_FILENAME}")
    print(f"   - 타겟 토픽: {VEHICLE_TOPIC_NAME}")

else:
    print(f"⚠️ 알 수 없는 차량 번호입니다: {target_vehicle_id}. 기본값(1)으로 설정합니다.")
    PATH_FILENAME = 'converted_path2.json'
    VEHICLE_TOPIC_NAME = '/CAV_01'

# ===============================================================
# [2] PID 제어기 (적분 제어 포함)
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
        
        # 적분 누적 제한 (Anti-Windup)
        if self.error_sum > 5.0: self.error_sum = 5.0
        if self.error_sum < -5.0: self.error_sum = -5.0
        
        i_term = self.ki * self.error_sum
        d_term = self.kd * (error - self.last_error) / self.dt
        self.last_error = error
        
        return p_term + i_term + d_term

# ===============================================================
# [3] 메인 드라이버 노드
# ===============================================================
class PrecisionDriver(Node):
    def __init__(self):
        super().__init__(f'driver_vehicle_{target_vehicle_id}') # 노드 이름도 유니크하게
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(PoseStamped, VEHICLE_TOPIC_NAME, self.pose_callback, qos_profile)
        self.accel_publisher = self.create_publisher(Accel, f'{VEHICLE_TOPIC_NAME}_accel', 10)
        
        self.path_x = []
        self.path_y = []
        self.load_path_file()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.is_pose_received = False

        # PID 튜닝값 (Kp, Ki, Kd)
        self.steering_pid = PIDController(kp=2.0, ki=0.05, kd=0.5, dt=0.05)
        self.timer = self.create_timer(0.05, self.drive_callback)

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

        # 1. 가장 가까운 점
        min_dist = float('inf')
        current_idx = 0
        for i in range(len(self.path_x)):
            dist = math.hypot(self.path_x[i] - self.current_x, self.path_y[i] - self.current_y)
            if dist < min_dist:
                min_dist = dist
                current_idx = i

        # 2. Look Ahead Point
        LOOK_AHEAD = 0.35 # 약간 늘림 (안정성)
        target_idx = current_idx
        for i in range(current_idx, len(self.path_x)):
            dist = math.hypot(self.path_x[i] - self.current_x, self.path_y[i] - self.current_y)
            if dist >= LOOK_AHEAD:
                target_idx = i
                break
        
        tx = self.path_x[target_idx]
        ty = self.path_y[target_idx]

        # 3. Heading Error 계산
        desired_yaw = math.atan2(ty - self.current_y, tx - self.current_x)
        yaw_err = desired_yaw - self.current_yaw
        
        while yaw_err > math.pi: yaw_err -= 2 * math.pi
        while yaw_err < -math.pi: yaw_err += 2 * math.pi

        # 4. PID 제어 + CTE 보정
        steer_cmd = self.steering_pid.compute(yaw_err)
        
        # CTE 보정 (경로 이탈 시 복귀)
        cte_gain = 0.5
        cte = min_dist * cte_gain
        if yaw_err < 0: cte = -cte # 방향에 따라 부호 조정
        
        final_steer = steer_cmd + cte

        # 제한
        final_steer = max(min(final_steer, 1.0), -1.0)
        
        cmd = Accel()
        cmd.linear.x = 0.15 # 속도
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

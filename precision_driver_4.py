import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import os
import math
from geometry_msgs.msg import Accel, PoseStamped

# === [설정 구역] ===
PATH_FILENAME = 'path_new.json'

# 1. 가변 속도 설정 (시뮬레이터 환경에 맞춰 조정 가능)
SPEED_STRAIGHT = 0.35      # 직선 구간에서의 시원한 가속 (m/s)
SPEED_CURVE = 0.12         # 곡선 구간에서 오차를 줄이기 위한 감속 (m/s)

# 2. 주시 거리 (Look Ahead)
LOOK_AHEAD_DISTANCE = 0.22  # 조향(핸들) 결정을 위한 거리
PREVIEW_DISTANCE = 0.60     # 속도(브레이크) 결정을 위해 미리 보는 거리

# 3. 제어 이득 (Gains)
STEER_GAIN = 2.6            # 핸들 반응성
CROSS_TRACK_GAIN = 1.8      # 경로로 복귀하려는 힘 (오차 감소의 핵심)

class PrecisionDriver4(Node):
    def __init__(self):
        super().__init__('precision_driver_4')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 구독 및 발행
        self.create_subscription(PoseStamped, '/CAV_01', self.pose_callback, qos_profile)
        self.accel_publisher = self.create_publisher(Accel, '/CAV_01_accel', 10)
        
        self.path_x, self.path_y = [], []
        self.load_path_file()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.is_pose_received = False
        self.current_vel = 0.15

        # 제어 주기 (50Hz로 높여서 더 정밀하게 제어)
        self.timer = self.create_timer(0.02, self.drive_callback)

    def load_path_file(self):
        if os.path.exists(PATH_FILENAME):
            with open(PATH_FILENAME, 'r') as f:
                data = json.load(f)
                self.path_x = data.get('X', [])
                self.path_y = data.get('Y', [])
                self.get_logger().info(f'📂 [precision_driver_4] {len(self.path_x)}개의 점 로드 완료')

    def pose_callback(self, msg):
        self.is_pose_received = True
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        q = msg.pose.orientation
        # 쿼터니언 -> Yaw 변환
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def drive_callback(self):
        if not self.is_pose_received or len(self.path_x) == 0:
            return

        # 1. 가장 가까운 점 찾기 (현재 오차 확인)
        min_dist = float('inf')
        current_idx = 0
        for i in range(len(self.path_x)):
            dist = math.sqrt((self.path_x[i]-self.current_x)**2 + (self.path_y[i]-self.current_y)**2)
            if dist < min_dist:
                min_dist = dist
                current_idx = i

        # 2. 조향용 목표점(Target) 및 속도 제어용 예견점(Preview) 찾기
        target_idx = current_idx
        preview_idx = current_idx
        
        for i in range(current_idx, len(self.path_x)):
            d = math.sqrt((self.path_x[i]-self.current_x)**2 + (self.path_y[i]-self.current_y)**2)
            if d >= LOOK_AHEAD_DISTANCE and target_idx == current_idx:
                target_idx = i
            if d >= PREVIEW_DISTANCE:
                preview_idx = i
                break

        # 3. 곡선 판단 (Curvature Detection)
        # 지금 가야 할 각도와 잠시 후(Preview) 가야 할 각도의 차이($\Delta \theta$) 계산
        angle_to_target = math.atan2(self.path_y[target_idx] - self.current_y, self.path_x[target_idx] - self.current_x)
        angle_to_preview = math.atan2(self.path_y[preview_idx] - self.path_y[target_idx], self.path_x[preview_idx] - self.path_x[target_idx])
        
        angle_diff = abs(angle_to_preview - angle_to_target)
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        angle_diff = abs(angle_diff)

        # 4. 가변 속도 결정 로직
        # 각도 차이가 0.1 rad(약 6도) 이상이면 곡선으로 진입 중이라 판단
        if angle_diff > 0.10:
            target_vel = SPEED_CURVE
        else:
            target_vel = SPEED_STRAIGHT

        # 오차가 너무 크면(예: 15cm 이상) 경로 복귀를 위해 속도를 강제로 낮춤
        if min_dist > 0.15:
            target_vel = SPEED_CURVE

        # 속도 변화를 부드럽게 (급가속/급브레이크 방지)
        self.current_vel = self.current_vel * 0.9 + target_vel * 0.1

        # 5. 조향 및 오차 보정 (로봇 기준 좌표계 변환)
        dx = self.path_x[target_idx] - self.current_x
        dy = self.path_y[target_idx] - self.current_y
        
        # 로봇 정면을 기준으로 목표점의 상대 위치(Local Y) 계산
        local_y = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)
        local_x = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)

        heading_error = math.atan2(local_y, local_x)
        
        # CTE(Cross Track Error) 보정: local_y 부호를 사용하여 경로 방향으로 복귀
        cte_sign = 1.0 if local_y > 0 else -1.0
        cte_correction = (min_dist * CROSS_TRACK_GAIN) * cte_sign

        final_steering = (heading_error * STEER_GAIN) + cte_correction
        
        # 조향 물리 한계 제한 (1.0 rad)
        final_steering = max(min(final_steering, 1.0), -1.0)

        # 6. 메시지 발행
        cmd = Accel()
        cmd.linear.x = self.current_vel
        cmd.angular.z = final_steering
        self.accel_publisher.publish(cmd)

        # 2초마다 상태 보고
        if self.get_clock().now().seconds_nanoseconds()[0] % 2 == 0:
            mode = "CURVE" if target_vel == SPEED_CURVE else "STRAIGHT"
            self.get_logger().info(f'[{mode}] Vel: {self.current_vel:.2f}m/s | Err: {min_dist*100:.1f}cm')

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionDriver4()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

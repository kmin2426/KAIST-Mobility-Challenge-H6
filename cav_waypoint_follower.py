#save code in Mobility_Challenge_Simulator/tool

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import os
from geometry_msgs.msg import Accel, PoseStamped

# =========================
# 설정(초보자 튜닝 포인트)
# =========================
WP_CSV = "full_path_21loop.csv"   # (중요) tool 폴더에 있는 파일명과 똑같이
TARGET_V = 0.4               # 속도(m/s). 빠르면 흔들림↑
LOOKAHEAD = 0.15                 # 주시거리(m). 빙글빙글이면 0.6~0.9로 ↑
MAX_STEER = 1.0             # 조향 제한(rad). 너무 작으면 코너 못돎

WINDOW = 100                     # idx 주변 탐색 범위(점프 방지)
TIMER_DT = 0.05                   # 20Hz (바꾸면 PID 튜닝도 같이 바꿔야함)

# =========================
# PID 조향 파라미터 (핵심)
# =========================
# ⭐ 추천 시작값 (빙글빙글/진동 줄이기)
Kp = 1.6
Ki = 0.0       # 처음엔 0으로 시작하세요. (I 넣으면 회전/폭주 가능)
Kd = 0.20

I_CLAMP = 0.40     # 적분 폭주 방지 (0.2~0.6)
D_LPF = 0.7       # 미분 필터 (0~1). 클수록 부드러움. 0.7~0.9 추천

# =========================
# 유틸 함수
# =========================
def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def yaw_from_quat(q):
    # 표준 quaternion -> yaw
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def load_waypoints(csv_path):
    pts = []
    with open(csv_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # 헤더(X,Y) 있으면 건너뜀
            if line[0].isalpha():
                continue
            a = line.split(",")
            if len(a) < 2:
                continue
            pts.append((float(a[0].strip()), float(a[1].strip())))
    if len(pts) < 5:
        raise RuntimeError("waypoints too short")
    return pts

# =========================
# 메인 노드
# =========================
class WaypointFollower(Node):
    def __init__(self, wp_file):
        super().__init__("cav_waypoint_follower")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(PoseStamped, "/CAV_01", self.on_pose, qos)
        self.pub = self.create_publisher(Accel, "/CAV_01_accel", 10)

        self.wps = load_waypoints(wp_file)
        self.n = len(self.wps)

        self.x = None
        self.y = None
        self.yaw = None
        self.pose_ok = False

        # 진행 인덱스 (빙글빙글 방지 핵심)
        self.idx = 0
        self.idx_initialized = False

        # PID 상태 변수
        self.err_i = 0.0
        self.err_prev = 0.0
        self.d_filt = 0.0
        self.dt = TIMER_DT

        self.timer = self.create_timer(self.dt, self.on_timer)  # 20Hz
        self.get_logger().info(f"[OK] Loaded {self.n} waypoints from: {wp_file}")

    def on_pose(self, msg: PoseStamped):
        self.pose_ok = True
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.orientation)

        # 처음 1번만: 전체에서 가장 가까운 점으로 idx 초기화
        if not self.idx_initialized:
            best_i = 0
            best_d = 1e18
            for i, (wx, wy) in enumerate(self.wps):
                dx = wx - self.x
                dy = wy - self.y
                d = dx*dx + dy*dy
                if d < best_d:
                    best_d = d
                    best_i = i
            self.idx = best_i
            self.idx_initialized = True

            # PID 초기화도 여기서 같이 (갑자기 튀는 것 방지)
            self.err_i = 0.0
            self.err_prev = 0.0
            self.d_filt = 0.0

            self.get_logger().info(f"[OK] Initialized idx={self.idx}")

    def nearest_in_window(self):
        # idx부터 앞으로 WINDOW 범위에서만 가장 가까운 점 찾기 (뒤로 점프 방지)
        start = self.idx
        end = min(self.idx + WINDOW, self.n)

        best_i = start
        best_d = 1e18
        for i in range(start, end):
            wx, wy = self.wps[i]
            dx = wx - self.x
            dy = wy - self.y
            d = dx*dx + dy*dy
            if d < best_d:
                best_d = d
                best_i = i
        return best_i, math.sqrt(best_d)

    def find_lookahead(self, base_idx):
        # base_idx부터 앞으로 보면서 LOOKAHEAD 이상 떨어진 점 선택
        for i in range(base_idx, self.n):
            wx, wy = self.wps[i]
            if math.hypot(wx - self.x, wy - self.y) >= LOOKAHEAD:
                return i
        return self.n - 1

    def on_timer(self):
        if not self.pose_ok or not self.idx_initialized:
            return

        near_i, near_dist = self.nearest_in_window()

        # idx는 앞으로만(뒤로 점프 방지)
        if near_i > self.idx:
            self.idx = near_i

        target_i = self.find_lookahead(self.idx)
        tx, ty = self.wps[target_i]

        # 목표 방향
        target_ang = math.atan2(ty - self.y, tx - self.x)
        err = wrap_pi(target_ang - self.yaw)  # heading error

        # =========================
        # PID 조향 제어 (핵심)
        # =========================
        p = Kp * err

        # I (적분 + anti-windup)
        self.err_i += err * self.dt
        if self.err_i > I_CLAMP:
            self.err_i = I_CLAMP
        elif self.err_i < -I_CLAMP:
            self.err_i = -I_CLAMP
        i = Ki * self.err_i

        # D (미분 + 저역통과필터)
        d_raw = (err - self.err_prev) / self.dt
        self.d_filt = D_LPF * self.d_filt + (1.0 - D_LPF) * d_raw
        d = Kd * self.d_filt
        self.err_prev = err

        steer = p + i + d
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        # publish
        cmd = Accel()
        cmd.linear.x = TARGET_V
        cmd.angular.z = steer
        self.pub.publish(cmd)

        # 2초마다 상태 로그
        now_sec = self.get_clock().now().seconds_nanoseconds()[0]
        if now_sec % 2 == 0:
            self.get_logger().info(
                f"idx={self.idx}/{self.n}  near={near_dist:.2f}m  err={err:.2f}rad  steer={steer:.2f}"
            )

def main():
    rclpy.init()

    # tool 폴더 기준 파일 경로 (이 파일이 tool 폴더에 있다고 가정)
    wp_path = os.path.join(os.path.dirname(__file__), WP_CSV)

    if not os.path.exists(wp_path):
        print(f"[ERROR] waypoint csv not found: {wp_path}")
        print("       WP_CSV 이름/위치(tool 폴더) 확인하세요.")
        return

    node = WaypointFollower(wp_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


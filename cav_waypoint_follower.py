import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import os
from geometry_msgs.msg import Accel, PoseStamped

# =========================
# 설정(초보자 튜닝 포인트)
# =========================
WP_CSV = "full_path_21loop.csv"  # 방금 만든 전체 경로
TARGET_V = 0.20                 # 속도 (너무 빠르면 흔들림)
LOOKAHEAD = 0.25                # 주시거리(빙글빙글이면 0.5~0.8로 올려)
STEER_GAIN = 1.6                # 조향 민감도
MAX_STEER = 1.0                 # 조향 제한
WINDOW = 150                    # idx 주변 탐색 범위(점프 방지)

def wrap_pi(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
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

        # "진행 인덱스" (빙글빙글 방지 핵심)
        self.idx = 0
        self.idx_initialized = False

        self.timer = self.create_timer(0.05, self.on_timer)  # 20Hz
        self.get_logger().info(f"Loaded {self.n} waypoints from: {wp_file}")

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
            self.get_logger().info(f"Initialized idx={self.idx}")

    def nearest_in_window(self):
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

        # 헤딩 오차
        target_ang = math.atan2(ty - self.y, tx - self.x)
        err = wrap_pi(target_ang - self.yaw)

        steer = STEER_GAIN * err
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        cmd = Accel()
        cmd.linear.x = TARGET_V
        cmd.angular.z = steer
        self.pub.publish(cmd)

def main():
    rclpy.init()
    # tool 폴더 기준 파일 경로
    wp_path = os.path.join(os.path.dirname(__file__), WP_CSV)
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


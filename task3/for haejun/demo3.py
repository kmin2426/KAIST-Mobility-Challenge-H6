#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import json
import os
import math
from geometry_msgs.msg import Accel, PoseStamped

# ============================================================
# [COMMON: path loader]
# ============================================================
def load_path_points(json_file):
    if not os.path.exists(json_file):
        return []
    with open(json_file, "r") as f:
        data = json.load(f)
    xs = data.get("x") or data.get("X")
    ys = data.get("y") or data.get("Y")
    if not xs or not ys:
        return []
    return [(float(x), float(y)) for x, y in zip(xs, ys)]


# ============================================================
# [Driver params] - 속도 0.7 고정
# ============================================================
FIXED_VELOCITY = 0.7

CURVE_PARAMS = {
    "vel": FIXED_VELOCITY,
    "look_ahead": 0.52,
    "kp": 6.0,
    "ki": 0.05,
    "kd": 1.0,
    "k_cte": 4.0
}

STRAIGHT_PARAMS = {
    "vel": FIXED_VELOCITY,
    "look_ahead": 0.52,
    "kp": 6.0,
    "ki": 0.05,
    "kd": 1.0,
    "k_cte": 4.0
}

WHEELBASE = 0.211
DIST_CENTER_TO_REAR = WHEELBASE / 2.0
TICK_RATE = 0.05

ACCEL_LIMIT = 0.8
DECEL_LIMIT = 3.0

class MapPredictionDriver(Node):

    
    def __init__(self, vehicle_id: int, path_filename: str):
        super().__init__(f'driver_vehicle_{vehicle_id}')
        self.vid = int(vehicle_id)
        self.last_idx = 0  # 인덱스 연속성 유지용
        self.PATH_FILENAME = path_filename
        self.TOPIC = f"/CAV_{self.vid:02d}"

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.path_pts = load_path_points(self.PATH_FILENAME)
        self.path_x = [p[0] for p in self.path_pts]
        self.path_y = [p[1] for p in self.path_pts]

        if not self.path_pts:
            self.get_logger().error(f"❌ [Car{self.vid}] Path missing: {self.PATH_FILENAME}")
        else:
            self.get_logger().info(f"✅ [Car{self.vid}] Path loaded: {self.PATH_FILENAME}")

        self.create_subscription(PoseStamped, self.TOPIC, self.pose_callback, qos_profile)
        self.accel_raw_pub = self.create_publisher(Accel, f"{self.TOPIC}_accel_raw", 10)

        self.curr_x, self.curr_y, self.curr_yaw = 0.0, 0.0, 0.0
        self.got_pose = False
        self.prev_err, self.int_err = 0.0, 0.0
        self.last_time = self.get_clock().now()
        self.current_vel_cmd = FIXED_VELOCITY
        self.create_timer(TICK_RATE, self.drive_loop)

    def pose_callback(self, msg):
        self.got_pose = True
        self.curr_yaw = float(msg.pose.orientation.z)
        self.curr_x = float(msg.pose.position.x) - (DIST_CENTER_TO_REAR * math.cos(self.curr_yaw))
        self.curr_y = float(msg.pose.position.y) - (DIST_CENTER_TO_REAR * math.sin(self.curr_yaw))

    def get_road_curvature(self, current_idx):
        num_pts = len(self.path_x)
        idx_now = current_idx
        idx_near = (current_idx + 50) % num_pts
        idx_far = (current_idx + 100) % num_pts

        dx1, dy1 = self.path_x[idx_near] - self.path_x[idx_now], self.path_y[idx_near] - self.path_y[idx_now]
        dx2, dy2 = self.path_x[idx_far] - self.path_x[idx_near], self.path_y[idx_far] - self.path_y[idx_near]
        
        diff = math.atan2(dy2, dx2) - math.atan2(dy1, dx1)
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        return abs(diff)
    def find_global_best_index(self):
        min_d = float('inf')
        best_idx = 0
        for idx in range(len(self.path_pts)):
            d = math.hypot(self.path_x[idx] - self.curr_x, self.path_y[idx] - self.curr_y)
            if d < min_d:
                min_d, best_idx = d, idx
        return best_idx

    def drive_loop(self):
        if not self.got_pose or not self.path_pts:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.001: return

        # ============================================================
        # [신규 추가] 위치 도약 감지 및 강제 재동기화 로직
        # ============================================================
        # 현재 저장된 인덱스(last_idx)와 실제 위치 간의 거리 계산
        actual_dist = math.hypot(self.path_x[self.last_idx] - self.curr_x, 
                                 self.path_y[self.last_idx] - self.curr_y)

        # 0.8m 이상 차이 나면 수동 이동으로 간주 (임계값은 환경에 따라 조정)
        if actual_dist > 0.8:
            self.get_logger().warn(f"⚠️ [Car{self.vid}] Jump Detected! Global re-localization triggered.")
            self.last_idx = self.find_global_best_index()
            self.int_err = 0.0  # 적분 오차 초기화 (급회전 방지)
            self.prev_err = 0.0 # 이전 오차 초기화
        # ============================================================

        num_pts = len(self.path_x)
        
        # 1) 현재 위치에서 가장 가까운 인덱스 탐색 (Circular)
        min_d = float('inf')
        best_idx = self.last_idx
        search_range = 50
        for i in range(self.last_idx - search_range, self.last_idx + search_range):
            idx = i % num_pts
            d = math.hypot(self.path_x[idx] - self.curr_x, self.path_y[idx] - self.curr_y)
            if d < min_d:
                min_d, best_idx = d, idx
        
        self.last_idx = best_idx
        curr_idx = best_idx
        # 2) 파라미터 결정
        road_curve_amount = self.get_road_curvature(curr_idx)
        if road_curve_amount < 0.15 and min_d < 0.4:
            params = STRAIGHT_PARAMS
            self.mode = "STRGT"
        else:
            params = CURVE_PARAMS
            self.mode = "CURVE"

        # 3) Look-ahead target 탐색
        target_idx = curr_idx
        for i in range(1, num_pts):
            idx = (curr_idx + i) % num_pts
            d = math.hypot(self.path_x[idx] - self.curr_x, self.path_y[idx] - self.curr_y)
            # 반드시 위에서 정의된 params를 사용해야 함
            if d >= params["look_ahead"]:
                target_idx = idx
                break

        tx, ty = self.path_x[target_idx], self.path_y[target_idx]
        desired_yaw = math.atan2(ty - self.curr_y, tx - self.curr_x)
        yaw_err = desired_yaw - self.curr_yaw
        while yaw_err > math.pi: yaw_err -= 2 * math.pi
        while yaw_err < -math.pi: yaw_err += 2 * math.pi

        # 4) PID 및 조향 계산
        self.int_err = max(-1.0, min(1.0, self.int_err + yaw_err * dt))
        p_term = params["kp"] * yaw_err
        i_term = params["ki"] * self.int_err
        d_term = params["kd"] * (yaw_err - self.prev_err) / dt
        cte_term = min_d * params["k_cte"] * (-1.0 if yaw_err < 0 else 1.0)

        final_steer = max(-1.0, min(1.0, float(p_term + i_term + d_term + cte_term)))
        self.prev_err = yaw_err

        # 5) 명령 발행
        cmd = Accel()
        cmd.linear.x = float(self.current_vel_cmd)
        cmd.angular.z = float(final_steer)
        self.accel_raw_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    drivers = [MapPredictionDriver(i, f"path3_{i}.json") for i in range(1, 5)]
    guardian = Problem3DualZoneGuardianMux()
    ex = MultiThreadedExecutor(num_threads=10)
    for d in drivers: ex.add_node(d)
    ex.add_node(guardian)
    try: ex.spin()
    except KeyboardInterrupt: pass
    finally:
        ex.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
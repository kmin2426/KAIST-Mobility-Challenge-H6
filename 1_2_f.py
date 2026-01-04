#!/usr/bin/env python3
import os
import csv
import math
import json

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Accel


# ============================================================
# [합류점(guardian) 코드 쪽으로 통일] IO helpers
# ============================================================
def load_dz_points(csv_file: str):
    pts = []
    if not os.path.exists(csv_file):
        return pts
    with open(csv_file, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                x = float(row[0]); y = float(row[1])
                if math.isfinite(x) and math.isfinite(y):
                    pts.append((x, y))
            except:
                pass
    return pts


def load_path_points(json_file: str):
    with open(json_file, "r") as f:
        data = json.load(f)

    pts = []
    if isinstance(data, dict):
        xs = data.get("x") or data.get("X")
        ys = data.get("y") or data.get("Y")
        if xs is None or ys is None:
            raise ValueError(f"Unsupported json dict format: {json_file}")
        for x, y in zip(xs, ys):
            x = float(x); y = float(y)
            if math.isfinite(x) and math.isfinite(y):
                pts.append((x, y))

    elif isinstance(data, list):
        for p in data:
            if isinstance(p, dict) and "x" in p and "y" in p:
                x = float(p["x"]); y = float(p["y"])
                if math.isfinite(x) and math.isfinite(y):
                    pts.append((x, y))
            elif isinstance(p, (list, tuple)) and len(p) >= 2:
                x = float(p[0]); y = float(p[1])
                if math.isfinite(x) and math.isfinite(y):
                    pts.append((x, y))
    else:
        raise ValueError(f"Unsupported json format: {json_file}")

    if len(pts) < 2:
        raise ValueError(f"Too few points in {json_file}")
    return pts


# ============================================================
# [합류점 코드 그대로] Geometry helpers
# ============================================================
def min_dist_to_points(x: float, y: float, pts):
    md = float("inf")
    for px, py in pts:
        d = math.hypot(x - px, y - py)
        if d < md:
            md = d
    return md


def build_cumulative_s(pts):
    s = [0.0]
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
    return s


def project_point_to_polyline_s(px, py, pts, s_cum):
    best_d2 = float("inf")
    best_s = 0.0

    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        vx, vy = (bx - ax), (by - ay)
        wx, wy = (px - ax), (py - ay)
        vv = vx * vx + vy * vy
        if vv <= 1e-12:
            continue

        t = (wx * vx + wy * vy) / vv
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0

        qx = ax + t * vx
        qy = ay + t * vy
        d2 = (px - qx) ** 2 + (py - qy) ** 2

        if d2 < best_d2:
            best_d2 = d2
            seg_len = math.sqrt(vv)
            best_s = s_cum[i] + t * seg_len

    return best_s


def dist2_point_to_polyline(px, py, pts):
    best = float("inf")
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        vx, vy = bx - ax, by - ay
        wx, wy = px - ax, py - ay
        vv = vx * vx + vy * vy
        if vv <= 1e-12:
            continue
        t = (wx * vx + wy * vy) / vv
        if t < 0.0: t = 0.0
        elif t > 1.0: t = 1.0
        qx = ax + t * vx
        qy = ay + t * vy
        d2 = (px - qx) ** 2 + (py - qy) ** 2
        if d2 < best:
            best = d2
    return best


# ============================================================
# [드라이버(A) 상수] (원래 값 그대로)
# ============================================================
ZONE_1_CSV = 'path1_1_rotary.csv'
ZONE_2_CSV = 'path1_2_zone.csv'

TARGET_VELOCITY = 0.48
LOOK_AHEAD_DISTANCE = 0.23
ZONE_TOLERANCE = 0.2

Kp = 5.0
Ki = 0.05
Kd = 2.3
K_cte = 6.0


# ============================================================
# [CODE A] ZonePriorityDriver (알고리즘 그대로, 1/2 동시 실행 가능하게만 리팩터)
# ============================================================
class ZonePriorityDriver(Node):
    def __init__(self, vehicle_id: int):
        self.vehicle_id = int(vehicle_id)

        if self.vehicle_id == 1:
            self.MY_PATH_FILE = 'path1_1.json'
            self.MY_TOPIC = '/CAV_01'
            self.OTHER_TOPIC = '/CAV_02'
            print(f"\n [차량 1] ready")
        else:
            self.MY_PATH_FILE = 'path1_2.json'
            self.MY_TOPIC = '/CAV_02'
            self.OTHER_TOPIC = '/CAV_01'
            print(f"\n [차량 2] ready")

        super().__init__(f'zone_driver_{self.vehicle_id}')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.accel_publisher = self.create_publisher(Accel, f'{self.MY_TOPIC}_accel', 10)
        self.create_subscription(PoseStamped, self.MY_TOPIC, self.pose_callback, qos_profile)
        self.create_subscription(PoseStamped, self.OTHER_TOPIC, self.other_pose_callback, qos_profile)

        # 경로 / zone 로드
        self.path_x, self.path_y = [], []
        self.zone1_points = []  # zone1
        self.zone2_points = []  # zone2

        self.load_my_path()

        # ✅ 중복 제거: CSV 로더는 guardian의 load_dz_points 사용
        self.zone1_points = load_dz_points(ZONE_1_CSV)
        self.zone2_points = load_dz_points(ZONE_2_CSV)

        if self.zone1_points:
            print(f"Zone 파일 로드: {ZONE_1_CSV}")
        else:
            print(f"Zone 파일 없음: {ZONE_1_CSV} (경고)")

        if self.zone2_points:
            print(f"Zone 파일 로드: {ZONE_2_CSV}")
        else:
            print(f"Zone 파일 없음: {ZONE_2_CSV} (경고)")

        # 상태
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.is_pose_received = False

        self.other_car_x = None
        self.other_car_y = None

        # PID 변수
        self.prev_error = 0.0
        self.integral_error = 0.0

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.drive_callback)
        self.log_counter = 0

    def load_my_path(self):
        if os.path.exists(self.MY_PATH_FILE):
            with open(self.MY_PATH_FILE, 'r') as f:
                data = json.load(f)
                self.path_x = data.get('X') or data.get('x') or []
                self.path_y = data.get('Y') or data.get('y') or []
            print(f"경로 로드 완료: {self.MY_PATH_FILE}")
        else:
            self.get_logger().error(f"경로 파일 없음: {self.MY_PATH_FILE}")

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

    def check_in_zone(self, x, y, zone_points):
        if not zone_points or x is None:
            return False
        for (zx, zy) in zone_points:
            dist = math.hypot(zx - x, zy - y)
            if dist < ZONE_TOLERANCE:
                return True
        return False

    def drive_callback(self):
        if not self.is_pose_received or not self.path_x:
            return

        # 1) Pure Pursuit 목표점 찾기
        min_dist = float('inf')
        current_idx = 0

        for i in range(len(self.path_x)):
            dist = math.hypot(self.path_x[i] - self.current_x, self.path_y[i] - self.current_y)
            if dist < min_dist:
                min_dist = dist
                current_idx = i

        target_idx = current_idx
        for i in range(current_idx, len(self.path_x)):
            if math.hypot(self.path_x[i] - self.current_x, self.path_y[i] - self.current_y) >= LOOK_AHEAD_DISTANCE:
                target_idx = i
                break

        tx = self.path_x[target_idx]
        ty = self.path_y[target_idx]

        # 2) 속도 제어
        final_velocity = TARGET_VELOCITY
        status_msg = "주행 중"

        # ✅ 원본 로직 그대로: 차량2만 zone 양보 로직 적용
        if self.vehicle_id == 2:
            am_i_in_zone = self.check_in_zone(self.current_x, self.current_y, self.zone2_points)
            is_opponent_in_zone = self.check_in_zone(self.other_car_x, self.other_car_y, self.zone1_points)

            if am_i_in_zone and is_opponent_in_zone:
                final_velocity = 0.05
                status_msg = "[양보] 상대방 통과 대기 중"
            elif am_i_in_zone:
                status_msg = "[진입] 상대방 없음 -> 통과"
            else:
                status_msg = "[일반]"

        # 3) 조향 제어 (PID + Stanley 보정)
        desired_yaw = math.atan2(ty - self.current_y, tx - self.current_x)
        yaw_err = desired_yaw - self.current_yaw

        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi

        self.integral_error = max(-1.0, min(1.0, self.integral_error + yaw_err * self.dt))

        p_term = Kp * yaw_err
        i_term = Ki * self.integral_error
        d_term = Kd * (yaw_err - self.prev_error) / self.dt

        cte_correction = min_dist * K_cte
        if yaw_err < 0:
            cte_correction = -cte_correction

        final_steering = max(min(p_term + i_term + d_term + cte_correction, 1.0), -1.0)
        self.prev_error = yaw_err

        # 4) 명령 전송
        cmd = Accel()
        cmd.linear.x = final_velocity
        cmd.angular.z = final_steering
        self.accel_publisher.publish(cmd)

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            print(f"[car{self.vehicle_id} {status_msg}] Vel: {final_velocity:.2f}, Idx: {current_idx}")


# ============================================================
# [CODE B] UpperRightCollisionGuardianSplitDZ (원본 그대로)
# ============================================================
class UpperRightCollisionGuardianSplitDZ(Node):
    def __init__(self):
        super().__init__("upper_right_collision_guardian_splitdz")

        self.DZ_RIGHT = "right_1_2.csv"
        self.DZ_UPPER_V1 = "upper_dz_v1.csv"  # 차량1용
        self.DZ_UPPER_V2 = "upper_dz_v2.csv"  # 차량2용

        self.PATH1 = "path1_1.json"
        self.PATH2 = "path1_2.json"

        self.NEAR_TOL_RIGHT = 0.40
        self.SAFETY_DIST_RIGHT = 1.50

        self.NEAR_TOL_UPPER = 0.80
        self.SAFETY_DIST_UPPER = 1.50
        self.UPPER_TIE_EPS = 0.10
        self.UPPER_TIE_YIELD_ID = 1
        self.UPPER_RELEASE_DIST = 2.20

        self.TICK = 0.02
        self.ACCEL_YIELD = 0.10

        self.ACCEL_RESUME = 0.05
        self.RESUME_PULSE_SEC = 1.5
        self.RESUME_PULSE_TICKS = int(self.RESUME_PULSE_SEC / self.TICK)

        self.RAMP_PER_SEC = 0.35
        self.MIN_ACCEL = 0.0

        self.dz_right = load_dz_points(self.DZ_RIGHT)
        self.dz_upper_v1 = load_dz_points(self.DZ_UPPER_V1)
        self.dz_upper_v2 = load_dz_points(self.DZ_UPPER_V2)

        if not self.dz_right:
            self.get_logger().warn(f"⚠️ RIGHT DZ empty/missing: {self.DZ_RIGHT}")
        if not self.dz_upper_v1:
            self.get_logger().warn(f"⚠️ UPPER V1 DZ empty/missing: {self.DZ_UPPER_V1}")
        if not self.dz_upper_v2:
            self.get_logger().warn(f"⚠️ UPPER V2 DZ empty/missing: {self.DZ_UPPER_V2}")

        self.path1_pts = load_path_points(self.PATH1)
        self.path2_pts = load_path_points(self.PATH2)
        self.path1_s = build_cumulative_s(self.path1_pts)
        self.path2_s = build_cumulative_s(self.path2_pts)

        upper_all = self.dz_upper_v1 + self.dz_upper_v2
        self.upper_merge = None
        if upper_all:
            best_score = float("inf")
            best_pt = None
            for zx, zy in upper_all:
                score = dist2_point_to_polyline(zx, zy, self.path1_pts) + dist2_point_to_polyline(zx, zy, self.path2_pts)
                if score < best_score:
                    best_score = score
                    best_pt = (zx, zy)
            self.upper_merge = best_pt
            self.get_logger().info(f"✅ UPPER merge point = {self.upper_merge}")

        self.s_merge_1 = None
        self.s_merge_2 = None
        if self.upper_merge is not None:
            mx, my = self.upper_merge
            self.s_merge_1 = project_point_to_polyline_s(mx, my, self.path1_pts, self.path1_s)
            self.s_merge_2 = project_point_to_polyline_s(mx, my, self.path2_pts, self.path2_s)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.p1 = None
        self.p2 = None
        self.sub1 = self.create_subscription(PoseStamped, "/CAV_01", self.cb1, qos)
        self.sub2 = self.create_subscription(PoseStamped, "/CAV_02", self.cb2, qos)
        self.pub1 = self.create_publisher(Accel, "/CAV_01_accel", 10)
        self.pub2 = self.create_publisher(Accel, "/CAV_02_accel", 10)

        self.cmd_accel = {1: 0.0, 2: 0.0}
        self.target_accel = {1: None, 2: None}
        self.resume_ticks_left = {1: 0, 2: 0}

        self.upper_lock_yield_id = None

        self.timer = self.create_timer(self.TICK, self.tick)
        self.get_logger().info("✅ Guardian started (UPPER split DZ: car1->V1, car2->V2)")

    def cb1(self, msg):
        self.p1 = (msg.pose.position.x, msg.pose.position.y)

    def cb2(self, msg):
        self.p2 = (msg.pose.position.x, msg.pose.position.y)

    def _publish_accel(self, vid: int, ax: float):
        msg = Accel()
        msg.linear.x = float(max(self.MIN_ACCEL, ax))
        msg.angular.z = 0.0
        if vid == 1:
            self.pub1.publish(msg)
        else:
            self.pub2.publish(msg)

    def _ramp_publish(self, vid: int):
        tgt = self.target_accel[vid]
        if tgt is None:
            return
        cur = self.cmd_accel[vid]
        step = self.RAMP_PER_SEC * self.TICK
        if tgt > cur:
            cur = min(tgt, cur + step)
        else:
            cur = max(tgt, cur - step)
        if cur < self.MIN_ACCEL:
            cur = self.MIN_ACCEL
        self.cmd_accel[vid] = cur
        self._publish_accel(vid, cur)

    def _start_resume_pulse(self, vid: int):
        self.resume_ticks_left[vid] = self.RESUME_PULSE_TICKS
        self.target_accel[vid] = self.ACCEL_RESUME

    def _cancel_resume(self, vid: int):
        self.resume_ticks_left[vid] = 0

    def _process_resume(self):
        for vid in (1, 2):
            if self.resume_ticks_left[vid] > 0:
                self._ramp_publish(vid)
                self.resume_ticks_left[vid] -= 1
                if self.resume_ticks_left[vid] <= 0:
                    self.target_accel[vid] = None

    def tick(self):
        if self.p1 is None or self.p2 is None:
            return

        x1, y1 = self.p1
        x2, y2 = self.p2
        d12 = math.hypot(x1 - x2, y1 - y2)

        self._process_resume()

        # RIGHT: 둘 다 RIGHT DZ 안이면 2 우선 => 1 감속
        if self.dz_right:
            near1_r = (min_dist_to_points(x1, y1, self.dz_right) <= self.NEAR_TOL_RIGHT)
            near2_r = (min_dist_to_points(x2, y2, self.dz_right) <= self.NEAR_TOL_RIGHT)

            if near1_r and near2_r and (d12 <= self.SAFETY_DIST_RIGHT):
                self._cancel_resume(1)
                self.target_accel[1] = self.ACCEL_YIELD
                self.target_accel[2] = None
                self._ramp_publish(1)
                return
            else:
                if self.target_accel[1] == self.ACCEL_YIELD and self.resume_ticks_left[1] == 0:
                    self._start_resume_pulse(1)

        # UPPER: 차량1은 V1, 차량2는 V2에 들어왔을 때만 작동
        if self.dz_upper_v1 and self.dz_upper_v2 and (self.s_merge_1 is not None) and (self.s_merge_2 is not None):
            near1_u = (min_dist_to_points(x1, y1, self.dz_upper_v1) <= self.NEAR_TOL_UPPER)
            near2_u = (min_dist_to_points(x2, y2, self.dz_upper_v2) <= self.NEAR_TOL_UPPER)

            if self.upper_lock_yield_id is not None:
                if (not (near1_u and near2_u)) or (d12 > self.UPPER_RELEASE_DIST):
                    yid = self.upper_lock_yield_id
                    self.upper_lock_yield_id = None
                    if self.target_accel.get(yid, None) == self.ACCEL_YIELD and self.resume_ticks_left[yid] == 0:
                        self._start_resume_pulse(yid)
                    return
                else:
                    yid = self.upper_lock_yield_id
                    self._cancel_resume(yid)
                    self.target_accel[yid] = self.ACCEL_YIELD
                    other = 1 if yid == 2 else 2
                    self.target_accel[other] = None
                    self._ramp_publish(yid)
                    return

            if near1_u and near2_u and (d12 <= self.SAFETY_DIST_UPPER):
                s_now_1 = project_point_to_polyline_s(x1, y1, self.path1_pts, self.path1_s)
                s_now_2 = project_point_to_polyline_s(x2, y2, self.path2_pts, self.path2_s)
                rem1 = max(0.0, self.s_merge_1 - s_now_1)
                rem2 = max(0.0, self.s_merge_2 - s_now_2)

                if rem1 < rem2 - self.UPPER_TIE_EPS:
                    yield_id = 2
                elif rem2 < rem1 - self.UPPER_TIE_EPS:
                    yield_id = 1
                else:
                    yield_id = self.UPPER_TIE_YIELD_ID

                self.upper_lock_yield_id = yield_id
                self._cancel_resume(yield_id)
                self.target_accel[yield_id] = self.ACCEL_YIELD
                other = 1 if yield_id == 2 else 2
                self.target_accel[other] = None
                self._ramp_publish(yield_id)
                return
            else:
                for vid in (1, 2):
                    if self.target_accel[vid] == self.ACCEL_YIELD and self.resume_ticks_left[vid] == 0:
                        self._start_resume_pulse(vid)


# ============================================================
# 실행: 한번에 car1+car2 driver + guardian 모두 실행
# ============================================================
def main():
    rclpy.init(args=None)

    node_driver_1 = ZonePriorityDriver(1)
    node_driver_2 = ZonePriorityDriver(2)
    node_guardian = UpperRightCollisionGuardianSplitDZ()

    ex = MultiThreadedExecutor()
    ex.add_node(node_driver_1)
    ex.add_node(node_driver_2)
    ex.add_node(node_guardian)

    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node_driver_1.destroy_node()
        node_driver_2.destroy_node()
        node_guardian.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


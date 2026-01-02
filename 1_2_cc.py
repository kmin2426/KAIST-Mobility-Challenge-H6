#!/usr/bin/env python3
import os
import csv
import math
import sys
import json
import importlib.util

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Accel, PoseStamped


# ============================================================
# [0] 유틸: 같은 py 파일을 "다른 모듈 이름"으로 2번 로드
# ============================================================
def load_driver_module_as(module_name: str, py_path: str, target_vehicle_id: int):
    old_argv = sys.argv[:]
    try:
        sys.argv = [py_path, str(target_vehicle_id)]
        spec = importlib.util.spec_from_file_location(module_name, py_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        sys.argv = old_argv


# ============================================================
# [1] DZ 로드/거리 계산
# ============================================================
def load_dz_points(csv_file: str):
    pts = []
    if not os.path.exists(csv_file):
        return pts

    with open(csv_file, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0]); y = float(row[1])
                if math.isfinite(x) and math.isfinite(y):
                    pts.append((x, y))
            except:
                pass
    return pts


def min_dist_to_dz(x: float, y: float, pts):
    md = float("inf")
    for zx, zy in pts:
        d = math.hypot(zx - x, zy - y)
        if d < md:
            md = d
    return md


# ============================================================
# [2] Path 로드 + arc-length(누적거리) + 투영(s) 계산
# ============================================================
def load_json_path_points(path_json: str):
    """지원 형태:
    - {"x":[...], "y":[...]} 또는 {"X":[...], "Y":[...]} 또는 [{"x":..,"y":..}, ...]
    """
    with open(path_json, "r") as f:
        data = json.load(f)

    pts = []
    if isinstance(data, dict):
        if "x" in data and "y" in data:
            xs, ys = data["x"], data["y"]
        elif "X" in data and "Y" in data:
            xs, ys = data["X"], data["Y"]
        else:
            raise ValueError(f"Unsupported json dict format: {path_json}")

        for x, y in zip(xs, ys):
            try:
                x = float(x); y = float(y)
                if math.isfinite(x) and math.isfinite(y):
                    pts.append((x, y))
            except:
                pass

    elif isinstance(data, list):
        for p in data:
            if isinstance(p, dict) and ("x" in p and "y" in p):
                x = float(p["x"]); y = float(p["y"])
                if math.isfinite(x) and math.isfinite(y):
                    pts.append((x, y))
    else:
        raise ValueError(f"Unsupported json format: {path_json}")

    if len(pts) < 2:
        raise ValueError(f"Too few points in {path_json}")
    return pts


def build_cumulative_s(pts):
    """pts: [(x,y), ...] -> s[i] = 0.. 누적거리"""
    s = [0.0]
    for i in range(1, len(pts)):
        x0, y0 = pts[i-1]
        x1, y1 = pts[i]
        s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
    return s


def project_point_to_polyline_s(px, py, pts, s_cum):
    """
    점 (px,py)을 폴리라인에 투영해서 가장 가까운 위치의 arc-length s를 반환.
    O(N)지만 포인트 수가 크지 않으면 충분.
    """
    best_d2 = float("inf")
    best_s = 0.0

    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        vx, vy = (bx - ax), (by - ay)
        wx, wy = (px - ax), (py - ay)
        vv = vx*vx + vy*vy
        if vv <= 1e-12:
            # degenerate
            t = 0.0
        else:
            t = (wx*vx + wy*vy) / vv
            if t < 0.0: t = 0.0
            if t > 1.0: t = 1.0

        qx = ax + t * vx
        qy = ay + t * vy
        d2 = (px - qx)**2 + (py - qy)**2

        if d2 < best_d2:
            seg_len = math.sqrt(vv)
            best_d2 = d2
            best_s = s_cum[i] + t * seg_len

    return best_s


# ============================================================
# [3] Guardian 노드 (DZ1 + DZ2)
#     - DZ2: arc-length로 "교차점까지 남은 거리" 비교해서 우선권 결정
# ============================================================
class MultiZoneGuardian(Node):
    def __init__(self):
        super().__init__("multi_zone_guardian")

        # ----------------------------
        # 파일명 (네 폴더 실제 이름으로 맞춰!)
        # ----------------------------
        self.DZ1_FILE = "right_1_2.csv"
        self.DZ2_FILE = "upper_1_2.csv"

        # DZ2 우선권 계산에 사용할 차량별 경로
        self.PATH1_JSON = "path1_1.json"   # 차량1 경로
        self.PATH2_JSON = "path1_2.json"  # 차량2 경로

        # ----------------------------
        # 파라미터
        # ----------------------------
        # DZ1
        self.DZ1_NEAR_TOL = 0.40
        self.DZ1_SAFETY_DIST = 1.20
        self.DZ1_EPS_Y = 0.06

        # DZ2
        self.DZ2_NEAR_TOL = 0.80      # near 안 잡히면 키워서 확인
        self.DZ2_SAFETY_DIST = 1.40

        # arc-length 비교 tie-break (m)
        self.DZ2_TIE_EPS = 0.10

        self.DEBUG_EVERY_N = 10

        # ----------------------------
        # QoS / 토픽
        # ----------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub1 = self.create_subscription(PoseStamped, "/CAV_01", self.cb_pose1, qos)
        self.sub2 = self.create_subscription(PoseStamped, "/CAV_02", self.cb_pose2, qos)

        self.pub1 = self.create_publisher(Accel, "/CAV_01_accel", 10)
        self.pub2 = self.create_publisher(Accel, "/CAV_02_accel", 10)

        self.p1 = None
        self.p2 = None

        # DZ 로드
        self.dz1 = load_dz_points(self.DZ1_FILE)
        self.dz2 = load_dz_points(self.DZ2_FILE)

        self.get_logger().info(f"✅ Loaded DZ1: {len(self.dz1)} pts ({self.DZ1_FILE})")
        self.get_logger().info(f"✅ Loaded DZ2: {len(self.dz2)} pts ({self.DZ2_FILE})")

        # ----------------------------
        # DZ2 교차점(근사): DZ2 점들의 "끝점"을 교차점으로 사용
        # - 기본은 x가 가장 큰 점
        # - 만약 교차점이 x최대가 아니면 여기 key를 바꿔야 함
        # ----------------------------
        if self.dz2:
            self.dz2_intersection_xy = max(self.dz2, key=lambda p: p[0])  # x 최대
            self.get_logger().info(f"✅ DZ2 intersection approx = {self.dz2_intersection_xy}")
        else:
            self.dz2_intersection_xy = None
            self.get_logger().warn("⚠️ DZ2 empty; DZ2 disabled")

        # ----------------------------
        # arc-length 준비 (차량별 경로 + 교차점 s 미리 계산)
        # ----------------------------
        try:
            self.path1_pts = load_json_path_points(self.PATH1_JSON)
            self.path2_pts = load_json_path_points(self.PATH2_JSON)
            self.path1_s = build_cumulative_s(self.path1_pts)
            self.path2_s = build_cumulative_s(self.path2_pts)

            if self.dz2_intersection_xy is not None:
                ix, iy = self.dz2_intersection_xy
                self.dz2_s_on_path1 = project_point_to_polyline_s(ix, iy, self.path1_pts, self.path1_s)
                self.dz2_s_on_path2 = project_point_to_polyline_s(ix, iy, self.path2_pts, self.path2_s)
                self.get_logger().info(
                    f"✅ DZ2 s_intersection: path1={self.dz2_s_on_path1:.2f}m, path2={self.dz2_s_on_path2:.2f}m"
                )
            else:
                self.dz2_s_on_path1 = None
                self.dz2_s_on_path2 = None

        except Exception as e:
            self.get_logger().error(f"❌ Path arc-length init failed: {e}")
            # DZ2 동적 우선권은 비활성화될 수 있음
            self.path1_pts = None
            self.path2_pts = None
            self.path1_s = None
            self.path2_s = None
            self.dz2_s_on_path1 = None
            self.dz2_s_on_path2 = None

        # ----------------------------
        # encounter 고정 (역정지 방지)
        # ----------------------------
        self.enc_zone = None
        self.enc_yield = None

        self._dbg = 0
        self.timer = self.create_timer(0.02, self.tick)  # 50Hz

    def cb_pose1(self, msg):
        self.p1 = (msg.pose.position.x, msg.pose.position.y)

    def cb_pose2(self, msg):
        self.p2 = (msg.pose.position.x, msg.pose.position.y)

    def publish_stop(self, cav_id: int):
        msg = Accel()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        if cav_id == 1:
            self.pub1.publish(msg)
        else:
            self.pub2.publish(msg)

    def in_zone(self, zone_name: str, x: float, y: float) -> bool:
        if zone_name == "DZ1" and self.dz1:
            return min_dist_to_dz(x, y, self.dz1) <= self.DZ1_NEAR_TOL
        if zone_name == "DZ2" and self.dz2:
            return min_dist_to_dz(x, y, self.dz2) <= self.DZ2_NEAR_TOL
        return False

    def dz2_remaining_arclen(self, cav_id: int, x: float, y: float) -> float:
        """해당 차량 경로 기준, 교차점까지 남은 arc-length(근사)"""
        if cav_id == 1:
            if self.path1_pts is None or self.dz2_s_on_path1 is None:
                return 999.0
            s_now = project_point_to_polyline_s(x, y, self.path1_pts, self.path1_s)
            return max(0.0, self.dz2_s_on_path1 - s_now)
        else:
            if self.path2_pts is None or self.dz2_s_on_path2 is None:
                return 999.0
            s_now = project_point_to_polyline_s(x, y, self.path2_pts, self.path2_s)
            return max(0.0, self.dz2_s_on_path2 - s_now)

    def tick(self):
        if self.p1 is None or self.p2 is None:
            return

        x1, y1 = self.p1
        x2, y2 = self.p2
        d12 = math.hypot(x1 - x2, y1 - y2)

        # ====================================================
        # [0] encounter 고정 유지 (역정지 방지)
        # - "둘 다 해당 DZ 안"인 동안에만 yield 정지 유지
        # ====================================================
        if self.enc_zone is not None and self.enc_yield is not None:
            in1 = self.in_zone(self.enc_zone, x1, y1)
            in2 = self.in_zone(self.enc_zone, x2, y2)
            if in1 and in2:
                self.publish_stop(self.enc_yield)
                return
            else:
                self.enc_zone = None
                self.enc_yield = None

        # ====================================================
        # [1] DZ1 (오른쪽 교차점) - 이전 로직 유지
        # ====================================================
        if self.dz1:
            md1_1 = min_dist_to_dz(x1, y1, self.dz1)
            md1_2 = min_dist_to_dz(x2, y2, self.dz1)
            near1 = (md1_1 <= self.DZ1_NEAR_TOL)
            near2 = (md1_2 <= self.DZ1_NEAR_TOL)

            if near1 and near2 and (d12 <= self.DZ1_SAFETY_DIST):
                if y1 < y2 - self.DZ1_EPS_Y:
                    yield_id = 2
                elif y2 < y1 - self.DZ1_EPS_Y:
                    yield_id = 1
                else:
                    yield_id = 2

                self.enc_zone = "DZ1"
                self.enc_yield = yield_id
                self.publish_stop(yield_id)
                return

        # ====================================================
        # [2] DZ2 (위쪽 교차점) - arc-length 우선권
        # ====================================================
        if self.dz2:
            md2_1 = min_dist_to_dz(x1, y1, self.dz2)
            md2_2 = min_dist_to_dz(x2, y2, self.dz2)
            near1 = (md2_1 <= self.DZ2_NEAR_TOL)
            near2 = (md2_2 <= self.DZ2_NEAR_TOL)

            self._dbg += 1
            if self._dbg % self.DEBUG_EVERY_N == 0:
                self.get_logger().info(
                    f"[DZ2] md1={md2_1:.2f} md2={md2_2:.2f} tol={self.DZ2_NEAR_TOL:.2f} "
                    f"d12={d12:.2f} near1={near1} near2={near2}"
                )

            if near1 and near2 and (d12 <= self.DZ2_SAFETY_DIST):
                rem1 = self.dz2_remaining_arclen(1, x1, y1)
                rem2 = self.dz2_remaining_arclen(2, x2, y2)

                # rem이 작은 쪽이 더 빨리 교차점 도착 => go, 반대 yield
                if rem1 < rem2 - self.DZ2_TIE_EPS:
                    yield_id = 2
                elif rem2 < rem1 - self.DZ2_TIE_EPS:
                    yield_id = 1
                else:
                    # tie-break: 기본은 2 go (원하면 1로 바꾸기)
                    yield_id = 2

                self.get_logger().info(f"[DZ2] rem1={rem1:.2f}m rem2={rem2:.2f}m => yield={yield_id}")

                self.enc_zone = "DZ2"
                self.enc_yield = yield_id
                self.publish_stop(yield_id)
                return


# ============================================================
# [4] main: 차량1/차량2 원본 드라이버 2개 + guardian 1개 실행
# ============================================================
def main():
    rclpy.init()

    base_py = os.path.join(os.path.dirname(__file__), "my_driver1_2_collision.py")

    mod1 = load_driver_module_as("driver_mod_1", base_py, 1)
    mod2 = load_driver_module_as("driver_mod_2", base_py, 2)

    driver1 = mod1.PriorityRotaryDriver()
    driver2 = mod2.PriorityRotaryDriver()

    guardian = MultiZoneGuardian()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(driver1)
    executor.add_node(driver2)
    executor.add_node(guardian)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        driver1.destroy_node()
        driver2.destroy_node()
        guardian.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


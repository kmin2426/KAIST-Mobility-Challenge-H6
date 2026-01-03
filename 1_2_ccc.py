#!/usr/bin/env python3
import os
import csv
import math
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Accel


# ============================================================
# IO helpers
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
        xs = data.get("x", data.get("X"))
        ys = data.get("y", data.get("Y"))
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
# Geometry helpers
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
# Guardian Node (RIGHT fixed priority, UPPER arc-length + lock)
# + Smooth resume pulse to avoid "snap back"
# ============================================================
class UpperRightCollisionGuardian(Node):
    def __init__(self):
        super().__init__("upper_right_collision_guardian")

        # ===== Files (같은 폴더) =====
        self.DZ_RIGHT = "right_1_2.csv"
        self.DZ_UPPER = "upper_1_2.csv"
        self.PATH1 = "path1_1.json"
        self.PATH2 = "path1_2.json"

        # ===== RIGHT (무조건 2 우선 => 1 양보) =====
        self.NEAR_TOL_RIGHT = 0.40
        self.SAFETY_DIST_RIGHT = 1.50

        # ===== UPPER (arc-length 우선) =====
        self.NEAR_TOL_UPPER = 0.80
        self.SAFETY_DIST_UPPER = 1.50
        self.UPPER_TIE_EPS = 0.10
        self.UPPER_TIE_YIELD_ID = 1  # tie면 1 감속(=2 우선)
        self.UPPER_RELEASE_DIST = 2.20  # lock 해제 히스테리시스

        # ===== Smooth control =====
        self.TICK = 0.02

        # 감속 목표(가속을 끊어 서서히 줄이게)
        self.ACCEL_YIELD = 0.10

        # ✅ “확 풀림” 방지: 위험 해제 후 잠깐 guardian가 부드럽게 복귀 가속을 주고, 그 다음 드라이버로 넘김
        self.ACCEL_RESUME = 0.15        # 복귀 가속(너무 크면 또 튐)
        self.RESUME_PULSE_SEC = 1.5     # 복귀 가속 유지 시간(초)
        self.RESUME_PULSE_TICKS = int(self.RESUME_PULSE_SEC / self.TICK)

        # ✅ ramp 더 부드럽게 (추천 0.3~0.6)
        self.RAMP_PER_SEC = 0.35

        # 음수 가속은 뒤로 폭주 이슈 때문에 막음
        self.MIN_ACCEL = 0.0

        # ===== Load data =====
        self.dz_right = load_dz_points(self.DZ_RIGHT)
        self.dz_upper = load_dz_points(self.DZ_UPPER)
        if not self.dz_right:
            self.get_logger().warn(f"⚠️ RIGHT DZ empty/missing: {self.DZ_RIGHT}")
        if not self.dz_upper:
            self.get_logger().warn(f"⚠️ UPPER DZ empty/missing: {self.DZ_UPPER}")

        self.path1_pts = load_path_points(self.PATH1)
        self.path2_pts = load_path_points(self.PATH2)
        self.path1_s = build_cumulative_s(self.path1_pts)
        self.path2_s = build_cumulative_s(self.path2_pts)

        # UPPER 교차점(merge) 자동 선택: 두 경로에 동시에 가까운 DZ 점
        self.upper_merge = None
        if self.dz_upper:
            best_score = float("inf")
            best_pt = None
            for zx, zy in self.dz_upper:
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

        # ===== ROS =====
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

        # accel state
        self.cmd_accel = {1: 0.0, 2: 0.0}
        self.target_accel = {1: None, 2: None}  # None이면 개입 안함

        # ✅ resume pulse state
        self.resume_ticks_left = {1: 0, 2: 0}

        # ✅ UPPER lock (토글 방지)
        self.upper_lock_yield_id = None

        self.timer = self.create_timer(self.TICK, self.tick)
        self.get_logger().info("✅ Guardian started (smooth ramp + resume pulse enabled)")

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
        """위험 해제 직후 '툭 풀림' 방지용: 일정 시간 ACCEL_RESUME로 ramp 후 제어 해제"""
        self.resume_ticks_left[vid] = self.RESUME_PULSE_TICKS
        self.target_accel[vid] = self.ACCEL_RESUME

    def _cancel_resume(self, vid: int):
        self.resume_ticks_left[vid] = 0

    def _process_resume_pulses(self):
        # resume 펄스가 켜져 있으면, 그 시간 동안은 guardian가 부드럽게 복귀 가속을 줌
        for vid in (1, 2):
            if self.resume_ticks_left[vid] > 0:
                self._ramp_publish(vid)
                self.resume_ticks_left[vid] -= 1
                if self.resume_ticks_left[vid] <= 0:
                    # 복귀 펄스 끝 -> 드라이버에 제어 넘김
                    self.target_accel[vid] = None

    def tick(self):
        if self.p1 is None or self.p2 is None:
            return

        x1, y1 = self.p1
        x2, y2 = self.p2
        d12 = math.hypot(x1 - x2, y1 - y2)

        # 0) 먼저 resume 펄스 처리 (단, 아래 hazard에서 override될 수 있음)
        self._process_resume_pulses()

        # ====================================================
        # 1) RIGHT: 무조건 2 우선 => 1 양보(감속)
        # ====================================================
        if self.dz_right:
            near1_r = (min_dist_to_points(x1, y1, self.dz_right) <= self.NEAR_TOL_RIGHT)
            near2_r = (min_dist_to_points(x2, y2, self.dz_right) <= self.NEAR_TOL_RIGHT)

            if near1_r and near2_r and (d12 <= self.SAFETY_DIST_RIGHT):
                # 감속 걸릴 때는 resume 펄스 취소
                self._cancel_resume(1)
                self.target_accel[1] = self.ACCEL_YIELD
                self.target_accel[2] = None
                self._ramp_publish(1)
                return
            else:
                # RIGHT에서 방금 풀린 경우: 1번 부드럽게 복귀
                # (단, 이미 다른 resume가 돌고 있으면 그대로)
                if self.target_accel[1] == self.ACCEL_YIELD:
                    self._start_resume_pulse(1)
                    return

        # ====================================================
        # 2) UPPER: arc-length + lock(토글 방지)
        # ====================================================
        if self.dz_upper and (self.s_merge_1 is not None) and (self.s_merge_2 is not None):
            near1_u = (min_dist_to_points(x1, y1, self.dz_upper) <= self.NEAR_TOL_UPPER)
            near2_u = (min_dist_to_points(x2, y2, self.dz_upper) <= self.NEAR_TOL_UPPER)

            # --- lock 유지 중이면 유지/해제 ---
            if self.upper_lock_yield_id is not None:
                if (not (near1_u and near2_u)) or (d12 > self.UPPER_RELEASE_DIST):
                    # lock 해제: 방금 양보하던 차량을 부드럽게 복귀
                    yid = self.upper_lock_yield_id
                    self.upper_lock_yield_id = None

                    # yield에서 풀릴 때 resume 펄스
                    if self.target_accel.get(yid, None) == self.ACCEL_YIELD:
                        self._start_resume_pulse(yid)
                    return
                else:
                    # lock 유지: 처음 정한 차량만 계속 감속
                    yid = self.upper_lock_yield_id
                    self._cancel_resume(yid)
                    self.target_accel[yid] = self.ACCEL_YIELD
                    other = 1 if yid == 2 else 2
                    self.target_accel[other] = None
                    self._ramp_publish(yid)
                    return

            # --- lock이 없으면, 조건 만족 시 새로 판단하고 lock 설정 ---
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
                    yield_id = self.UPPER_TIE_YIELD_ID  # tie면 1 감속

                self.upper_lock_yield_id = yield_id

                self._cancel_resume(yield_id)
                self.target_accel[yield_id] = self.ACCEL_YIELD
                other = 1 if yield_id == 2 else 2
                self.target_accel[other] = None
                self._ramp_publish(yield_id)
                return
            else:
                # UPPER에 없는데 과거에 yield 목표가 남아있으면 부드럽게 복귀
                for vid in (1, 2):
                    if self.target_accel[vid] == self.ACCEL_YIELD and self.resume_ticks_left[vid] == 0:
                        self._start_resume_pulse(vid)
                return

        # ====================================================
        # 3) 아무 위험도 아니면: yield 상태였던 차는 resume로 부드럽게 풀어주고,
        #    resume 끝나면 자동으로 None 처리됨.
        # ====================================================
        for vid in (1, 2):
            if self.target_accel[vid] == self.ACCEL_YIELD and self.resume_ticks_left[vid] == 0:
                self._start_resume_pulse(vid)


def main():
    rclpy.init()
    node = UpperRightCollisionGuardian()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


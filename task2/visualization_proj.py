#!/usr/bin/env python3
import json
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker


# =========================
# Config
# =========================
PATH_FILES = [
    '/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/path_2_1.json',
    '/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/path_2_2.json',
    '/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/path_2_3.json',
    '/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/path_2_4.json',
]
LANE_COUNT = 4

# ---- Topics ----
HV_IDS = list(range(19, 37))   # /HV_19 ... /HV_36
CAV_ENABLE = True
CAV_TOPIC = "/CAV_01"             # 너 환경에 맞게 수정 가능
CAV_ID = 0                     # marker id용

FRAME_ID = "map"
FRAME_RATE = 20.0  # Hz

# ---- Markers ----
PATH_LINE_WIDTH = 0.03

POINT_P_SCALE = 0.08
POINT_Q_SCALE = 0.06
LINE_PQ_WIDTH = 0.02

ID_TEXT_ENABLE = True
ID_TEXT_Z = 0.35
ID_TEXT_SIZE = 0.18

# ---- Projection search ----
NEAREST_WINDOW = 200
GLOBAL_WINDOW = 10**9
RECOVER_DIST = 0.6
JUMP_WARN_DIST = 1.5

# ---- Republish paths ----
PATH_REPUB_PERIOD = 1.0


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


# =========================
# State
# =========================
@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    valid: bool = False
    seg_hint: Optional[Dict[int, int]] = None

    def __post_init__(self):
        if self.seg_hint is None:
            self.seg_hint = {}


# =========================
# Node
# =========================
class ProjectionVizNode(Node):
    def __init__(self):
        super().__init__("projection_viz_hv_cav")

        # Load paths + preprocess arc-length
        self.paths = [self.load_path(p) for p in PATH_FILES]
        for p in self.paths:
            p["s"] = self.preprocess_path(p["x"], p["y"])

        # States
        self.hvs: Dict[int, VehicleState] = {hv_id: VehicleState() for hv_id in HV_IDS}
        self.cav: VehicleState = VehicleState()

        # Subscribers
        self.hv_subs = []
        for hv_id in HV_IDS:
            topic = f"/HV_{hv_id}"
            sub = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, hv_id=hv_id: self.hv_pose_callback(hv_id, msg),
                qos_profile_sensor_data,
            )
            self.hv_subs.append(sub)

        if CAV_ENABLE:
            self.cav_sub = self.create_subscription(
                PoseStamped,
                CAV_TOPIC,
                self.cav_pose_callback,
                qos_profile_sensor_data,
            )

        # Publishers
        self.path_pub = self.create_publisher(Marker, "/viz/paths", 10)
        self.viz_pub = self.create_publisher(Marker, "/viz/projection", 50)

        # Timers
        self.last_path_pub = 0.0
        self.timer = self.create_timer(1.0 / FRAME_RATE, self.loop)

        self.get_logger().info("ProjectionVizNode started (HV + CAV).")

    # ---------- Path loading ----------
    def load_path(self, file: str) -> Dict[str, List[float]]:
        with open(file, "r") as f:
            data = json.load(f)
        xs = list(map(float, data["X"]))
        ys = list(map(float, data["Y"]))
        if len(xs) != len(ys) or len(xs) < 2:
            raise RuntimeError(f"Bad path file: {file}")
        return {"x": xs, "y": ys}

    def preprocess_path(self, xs: List[float], ys: List[float]) -> List[float]:
        s = [0.0]
        for i in range(1, len(xs)):
            s.append(s[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))
        return s

    # ---------- Callbacks ----------
    def hv_pose_callback(self, hv_id: int, msg: PoseStamped):
        hv = self.hvs[hv_id]
        hv.x = msg.pose.position.x
        hv.y = msg.pose.position.y
        hv.valid = True

    def cav_pose_callback(self, msg: PoseStamped):
        self.cav.x = msg.pose.position.x
        self.cav.y = msg.pose.position.y
        self.cav.valid = True

    # =========================
    # Projection core (네 로직)
    # =========================
    def project_to_lane(
        self,
        lane_idx: int,
        x: float,
        y: float,
        hint: int = 0,
        window: Optional[int] = None,
    ) -> Tuple[float, float, int, float, float, float, float]:
        """
        Returns:
          s_proj, d_proj, seg_i, u, abs_d, qx, qy
        """
        if window is None:
            window = NEAREST_WINDOW

        px = self.paths[lane_idx]["x"]
        py = self.paths[lane_idx]["y"]
        ps = self.paths[lane_idx]["s"]

        n = len(px)
        if n < 2:
            return 0.0, 0.0, 0, 0.0, 1e9, 0.0, 0.0

        hint = int(clamp(hint, 0, n - 2))
        a = max(0, hint - window)
        b = min(n - 1, hint + window + 1)

        best_dist2 = 1e30
        best_i = a
        best_u = 0.0
        best_s = ps[a]
        best_d = 0.0
        best_qx = px[a]
        best_qy = py[a]

        for i in range(a, b - 1):
            x1, y1 = px[i], py[i]
            x2, y2 = px[i + 1], py[i + 1]

            vx = x2 - x1
            vy = y2 - y1
            seg_len2 = vx * vx + vy * vy
            if seg_len2 < 1e-12:
                continue

            wx = x - x1
            wy = y - y1
            u = clamp((wx * vx + wy * vy) / seg_len2, 0.0, 1.0)

            qx = x1 + u * vx
            qy = y1 + u * vy

            dx = x - qx
            dy = y - qy
            dist2 = dx * dx + dy * dy

            if dist2 < best_dist2:
                best_dist2 = dist2
                best_i = i
                best_u = u
                best_qx = qx
                best_qy = qy
                best_s = ps[i] + u * (ps[i + 1] - ps[i])
                cross = vx * dy - vy * dx
                mag = math.sqrt(dist2)
                best_d = mag if cross > 0 else -mag

        return best_s, best_d, best_i, best_u, abs(best_d), best_qx, best_qy

    # ---------- robust wrapper ----------
    def robust_project(
        self,
        veh: VehicleState,
        lane_idx: int,
        x: float,
        y: float,
    ) -> Tuple[float, float, int, float, float, float, float, str]:
        hint_opt = veh.seg_hint.get(lane_idx, None)

        # 1) global init
        if hint_opt is None:
            s, d, seg_i, u, abs_d, qx, qy = self.project_to_lane(
                lane_idx, x, y, hint=0, window=GLOBAL_WINDOW
            )
            veh.seg_hint[lane_idx] = seg_i
            return s, d, seg_i, u, abs_d, qx, qy, "global_init"

        # 2) local scan
        s, d, seg_i, u, abs_d, qx, qy = self.project_to_lane(
            lane_idx, x, y, hint=hint_opt, window=NEAREST_WINDOW
        )
        mode = "local"

        # 3) recover
        if abs_d > RECOVER_DIST:
            s2, d2, seg_i2, u2, abs_d2, qx2, qy2 = self.project_to_lane(
                lane_idx, x, y, hint=0, window=GLOBAL_WINDOW
            )
            if abs_d2 < abs_d:
                s, d, seg_i, u, abs_d, qx, qy = s2, d2, seg_i2, u2, abs_d2, qx2, qy2
                mode = "recover_global"

        veh.seg_hint[lane_idx] = seg_i
        return s, d, seg_i, u, abs_d, qx, qy, mode

    # =========================
    # RViz publishing
    # =========================
    def publish_paths(self):
        now = self.get_clock().now().to_msg()

        colors = [
            (1.0, 0.0, 0.0),  # lane0
            (0.0, 1.0, 0.0),  # lane1
            (0.0, 0.0, 1.0),  # lane2
            (1.0, 1.0, 1.0),  # lane3
        ]

        for lane_idx, path in enumerate(self.paths):
            m = Marker()
            m.header.frame_id = FRAME_ID
            m.header.stamp = now
            m.ns = "lane_path"
            m.id = lane_idx
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = PATH_LINE_WIDTH
            r, g, b = colors[lane_idx] if lane_idx < len(colors) else (1.0, 1.0, 1.0)
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0

            m.points = []
            for x, y in zip(path["x"], path["y"]):
                p = Point()
                p.x, p.y, p.z = x, y, 0.0
                m.points.append(p)

            self.path_pub.publish(m)

    def publish_current_best(self, prefix: str, veh_id: int, lane_idx: int, px: float, py: float, qx: float, qy: float):
        now = self.get_clock().now().to_msg()

        lane_colors = [
            (1.0, 0.4, 0.4),
            (0.4, 1.0, 0.4),
            (0.4, 0.4, 1.0),
            (1.0, 1.0, 1.0),
        ]
        lr, lg, lb = lane_colors[lane_idx] if lane_idx < 4 else (1.0, 1.0, 1.0)

        # ===== IMPORTANT: 완전 분리된 ID 체계 =====
        base = 0 if prefix == "hv" else 100000
        ID_P = base + veh_id * 10 + 1
        ID_Q = base + veh_id * 10 + 2
        ID_L = base + veh_id * 10 + 3

        # P (vehicle point)
        mp = Marker()
        mp.header.frame_id = FRAME_ID
        mp.header.stamp = now
        mp.ns = f"{prefix}_point"
        mp.id = ID_P
        mp.type = Marker.SPHERE
        mp.action = Marker.ADD
        mp.pose.position.x = px
        mp.pose.position.y = py
        mp.pose.position.z = 0.10
        mp.scale.x = mp.scale.y = mp.scale.z = POINT_P_SCALE
        if prefix == "hv":
            mp.color.r, mp.color.g, mp.color.b = 1.0, 0.0, 0.0  # HV red
        else:
            mp.color.r, mp.color.g, mp.color.b = 1.0, 1.0, 0.0  # CAV yellow
        mp.color.a = 1.0
        self.viz_pub.publish(mp)

        # Q (projection)
        mq = Marker()
        mq.header.frame_id = FRAME_ID
        mq.header.stamp = now
        mq.ns = f"{prefix}_proj_point"
        mq.id = ID_Q
        mq.type = Marker.SPHERE
        mq.action = Marker.ADD
        mq.pose.position.x = qx
        mq.pose.position.y = qy
        mq.pose.position.z = 0.10
        mq.scale.x = mq.scale.y = mq.scale.z = POINT_Q_SCALE
        mq.color.r, mq.color.g, mq.color.b, mq.color.a = lr, lg, lb, 1.0
        self.viz_pub.publish(mq)

        # Line P->Q
        ml = Marker()
        ml.header.frame_id = FRAME_ID
        ml.header.stamp = now
        ml.ns = f"{prefix}_proj_line"
        ml.id = ID_L
        ml.type = Marker.LINE_LIST
        ml.action = Marker.ADD
        ml.scale.x = LINE_PQ_WIDTH
        ml.color.r, ml.color.g, ml.color.b, ml.color.a = lr, lg, lb, 1.0

        p1 = Point()
        p1.x, p1.y, p1.z = px, py, 0.05
        p2 = Point()
        p2.x, p2.y, p2.z = qx, qy, 0.05
        ml.points = [p1, p2]
        self.viz_pub.publish(ml)

    def publish_id_text(self, prefix: str, veh_id: int, lane_idx: int, x: float, y: float):
        if not ID_TEXT_ENABLE:
            return

        now = self.get_clock().now().to_msg()

        base = 0 if prefix == "hv" else 100000
        ID_T = base + veh_id * 10 + 9  # 텍스트는 9

        mt = Marker()
        mt.header.frame_id = FRAME_ID
        mt.header.stamp = now
        mt.ns = f"{prefix}_id_text"
        mt.id = ID_T
        mt.type = Marker.TEXT_VIEW_FACING
        mt.action = Marker.ADD
        mt.pose.position.x = x
        mt.pose.position.y = y
        mt.pose.position.z = ID_TEXT_Z
        mt.scale.z = ID_TEXT_SIZE
        mt.color.r = mt.color.g = mt.color.b = 1.0
        mt.color.a = 1.0

        name = "HV" if prefix == "hv" else "CAV"
        mt.text = f"{name}{veh_id}  lane {lane_idx}"
        self.viz_pub.publish(mt)

    # =========================
    # Loop
    # =========================
    def loop(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Republish paths periodically (RViz late-join safe)
        if (now_sec - self.last_path_pub) > PATH_REPUB_PERIOD:
            self.publish_paths()
            self.last_path_pub = now_sec

        # ---- HVs ----
        for hv_id, hv in self.hvs.items():
            if not hv.valid:
                continue

            lane_results = []
            for lane_idx in range(LANE_COUNT):
                s, d, seg_i, u, abs_d, qx, qy, mode = self.robust_project(hv, lane_idx, hv.x, hv.y)
                lane_results.append((abs_d, lane_idx, qx, qy, s, d, seg_i, mode))

            lane_results.sort(key=lambda t: t[0])
            best_absd, best_lane, qx, qy, s, d, seg_i, mode = lane_results[0]

            if best_absd > JUMP_WARN_DIST:
                self.get_logger().warn(
                    f"[JUMP?] HV{hv_id} pos=({hv.x:.2f},{hv.y:.2f}) best=lane{best_lane} abs_d={best_absd:.2f} mode={mode}"
                )

            self.publish_current_best("hv", hv_id, best_lane, hv.x, hv.y, qx, qy)
            self.publish_id_text("hv", hv_id, best_lane, hv.x, hv.y)

        # ---- CAV ----
        if CAV_ENABLE and self.cav.valid:
            lane_results = []
            for lane_idx in range(LANE_COUNT):
                s, d, seg_i, u, abs_d, qx, qy, mode = self.robust_project(self.cav, lane_idx, self.cav.x, self.cav.y)
                lane_results.append((abs_d, lane_idx, qx, qy, s, d, seg_i, mode))

            lane_results.sort(key=lambda t: t[0])
            best_absd, best_lane, qx, qy, s, d, seg_i, mode = lane_results[0]

            if best_absd > JUMP_WARN_DIST:
                self.get_logger().warn(
                    f"[JUMP?] CAV pos=({self.cav.x:.2f},{self.cav.y:.2f}) best=lane{best_lane} abs_d={best_absd:.2f} mode={mode}"
                )

            self.publish_current_best("cav", CAV_ID, best_lane, self.cav.x, self.cav.y, qx, qy)
            self.publish_id_text("cav", CAV_ID, best_lane, self.cav.x, self.cav.y)


def main():
    rclpy.init()
    node = ProjectionVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import math
from typing import Dict, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

from projection import ProjectionManager, EgoState


# ================== 설정 ==================
HV_IDS = (19, 20)
CAV_IDS = (1, 2, 3, 4)

# lane index (projection.py의 PATH_FILES 순서 기준)
LANE_HV_ZONE_2 = 0   # path_hv_2 (2사분면)
LANE_HV_ZONE_3 = 1   # path_hv_3 (3사분면)

LANE_CAV1_ZONE = 3
LANE_CAV2_ZONE = 4
LANE_CAV3_ZONE = 5
LANE_CAV4_ZONE = 6

# CAV별 "자기 zone lane"
CAV_ZONE_LANE = {
    1: LANE_CAV1_ZONE,
    2: LANE_CAV2_ZONE,
    3: LANE_CAV3_ZONE,
    4: LANE_CAV4_ZONE,
}

# CAV별 "HV 트리거 존 lane"
HV_TRIGGER_ZONE_LANE = {
    1: LANE_HV_ZONE_3,
    4: LANE_HV_ZONE_3,
    2: LANE_HV_ZONE_2,
    3: LANE_HV_ZONE_2,
}

# ================== 임계값 ==================
HV_ZONE_ABS_D = 0.10    # HV가 해당 lane path에 충분히 붙었는지
CAV_ZONE_ABS_D = 0.10   # CAV가 자기 zone lane 안인지

# 출력 속도
MIN_SPEED = 0.0
FREE_SPEED = 99.0

# 로그 주기
LOG_PERIOD = 0.2

# ================== HV 존 s-윈도우(중요) ==================
# "center" 주변 +- "half" 범위만 존으로 인정 (원형 s 거리 사용)
# 처음엔 대충 넣고, 로그의 DBG에서 s 값 보고 맞춰.
HV_ZONE_S = {
    LANE_HV_ZONE_2: {"center": 0.0, "half": 3.0},  # TODO: 맞추기
    LANE_HV_ZONE_3: {"center": 0.0, "half": 3.0},  # TODO: 맞추기
}

# 디버그: HV가 각 lane에 대해 (abs_d가 작을 때) s를 보여주고 "추천 center" 찍음
DEBUG_SUGGEST = True
SUGGEST_ABS_D = 0.10   # 이 abs_d 이하일 때 s를 추천 후보로 찍음


class HVTailgateNode(Node):
    def __init__(self):
        super().__init__("hv_tailgate_node")

        self.pm = ProjectionManager(hv_ids=HV_IDS)
        self.cavs: Dict[int, EgoState] = {vid: EgoState() for vid in CAV_IDS}

        self.pub_gate_speed: Dict[int, rclpy.publisher.Publisher] = {}
        for vid in CAV_IDS:
            self.pub_gate_speed[vid] = self.create_publisher(
                Float32, f"/CAV_{vid:02d}_gate_speed", 10
            )

        # HV subs
        self.create_subscription(
            PoseStamped, "/HV_19",
            lambda msg: self.pm.update_hv_pose(19, msg),
            qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, "/HV_20",
            lambda msg: self.pm.update_hv_pose(20, msg),
            qos_profile_sensor_data
        )

        # CAV subs
        for vid in CAV_IDS:
            self.create_subscription(
                PoseStamped, f"/CAV_{vid:02d}",
                self._make_cav_cb(vid),
                qos_profile_sensor_data
            )

        self.create_timer(LOG_PERIOD, self.on_timer)

        self.get_logger().info(
            "[hv.py] Tailgating (CAV zone gate: HV trig IN->FREE else STOP)\n"
            f"  HV_ZONE_ABS_D={HV_ZONE_ABS_D:.2f} | CAV_ZONE_ABS_D={CAV_ZONE_ABS_D:.2f}\n"
            f"  MIN_SPEED={MIN_SPEED:.2f} | FREE_SPEED={FREE_SPEED:.2f}\n"
            f"  lane{LANE_HV_ZONE_2} HV_ZONE_S={HV_ZONE_S[LANE_HV_ZONE_2]}\n"
            f"  lane{LANE_HV_ZONE_3} HV_ZONE_S={HV_ZONE_S[LANE_HV_ZONE_3]}\n"
            f"  CAV2,3 -> HV_ZONE lane{LANE_HV_ZONE_2} | CAV1,4 -> HV_ZONE lane{LANE_HV_ZONE_3}"
        )

    def _make_cav_cb(self, vid: int):
        def cb(msg: PoseStamped):
            cav = self.cavs[vid]
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            x = msg.pose.position.x
            y = msg.pose.position.y

            if cav.valid:
                cav.prev_time = cav.t
                cav.prev_x = cav.x
                cav.prev_y = cav.y
                dt = t - (cav.prev_time if cav.prev_time is not None else t)
                if dt > 1e-3 and cav.prev_x is not None and cav.prev_y is not None:
                    cav.speed = math.hypot(x - cav.prev_x, y - cav.prev_y) / dt

            cav.t = t
            cav.x = x
            cav.y = y
            cav.valid = True

        return cb

    def _proj_absd(self, lane_idx: int, x: float, y: float) -> Tuple[float, float, float, str]:
        # robust_project returns: s, d, seg_i, u, abs_d, qx, qy, mode
        tmp = EgoState()
        tmp.valid = True
        s, d, seg_i, u, abs_d, qx, qy, mode = self.pm.robust_project(tmp, lane_idx, x, y)
        return float(s), float(d), float(abs_d), str(mode)

    def _circ_dist(self, s: float, s0: float, L: float) -> float:
        d = abs(s - s0)
        return min(d, L - d)

    def _in_s_window(self, lane_idx: int, s: float) -> bool:
        z = HV_ZONE_S[lane_idx]
        center_s = float(z["center"])
        half = float(z["half"])
        L = float(self.pm.paths[lane_idx]["s"][-1])
        return self._circ_dist(s, center_s, L) <= half

    def _hv_in_zone_lane(self, lane_idx: int) -> Tuple[bool, str]:
        """HV가 해당 '존 lane' 안이면 True (HV 둘 중 하나라도)"""
        who = []
        for hid in HV_IDS:
            hv = self.pm.hvs[hid]
            if not hv.valid:
                continue

            s, d, abs_d, mode = self._proj_absd(lane_idx, hv.x, hv.y)
            ok = (abs_d <= HV_ZONE_ABS_D) and self._in_s_window(lane_idx, s)

            if ok:
                who.append(f"HV{hid}(s={s:.2f},abs_d={abs_d:.2f},{mode})")

        return (len(who) > 0), (", ".join(who) if who else "-")

    def _cav_in_own_zone(self, vid: int) -> Tuple[bool, float, str]:
        """CAV가 자기 zone 안이면 True"""
        cav = self.cavs[vid]
        if not cav.valid:
            return False, 999.0, "no_pose"

        lane_idx = CAV_ZONE_LANE[vid]
        s, d, abs_d, mode = self._proj_absd(lane_idx, cav.x, cav.y)
        return (abs_d <= CAV_ZONE_ABS_D), abs_d, mode

    def _publish_gate_speed(self, vid: int, v: float):
        msg = Float32()
        msg.data = float(v)
        self.pub_gate_speed[vid].publish(msg)

    def _debug_s_suggest(self) -> Optional[str]:
        if not DEBUG_SUGGEST:
            return None

        lines = []
        # "이 HV 위치에서 lane0/lane1 각각 투영 s"를 같이 보여줌
        for hid in HV_IDS:
            hv = self.pm.hvs[hid]
            if not hv.valid:
                continue

            s0, d0, a0, m0 = self._proj_absd(LANE_HV_ZONE_2, hv.x, hv.y)
            s1, d1, a1, m1 = self._proj_absd(LANE_HV_ZONE_3, hv.x, hv.y)

            tag0 = "CAND" if a0 <= SUGGEST_ABS_D else "----"
            tag1 = "CAND" if a1 <= SUGGEST_ABS_D else "----"

            lines.append(
                f"[DBG] HV{hid} | "
                f"lane{LANE_HV_ZONE_2}: s={s0:.2f}, abs_d={a0:.2f} {tag0} | "
                f"lane{LANE_HV_ZONE_3}: s={s1:.2f}, abs_d={a1:.2f} {tag1}"
            )

        return "\n".join(lines) if lines else None

    def on_timer(self):
        # HV 존 상태 계산 (abs_d + s-window)
        hv_in_zone2, zone2_who = self._hv_in_zone_lane(LANE_HV_ZONE_2)
        hv_in_zone3, zone3_who = self._hv_in_zone_lane(LANE_HV_ZONE_3)

        gate_speed: Dict[int, float] = {vid: FREE_SPEED for vid in CAV_IDS}

        lines = []
        dbg = self._debug_s_suggest()
        if dbg:
            lines.append(dbg)
            lines.append("---")

        lines.append(
            f"HV_ZONE lane{LANE_HV_ZONE_2}: {'IN ' if hv_in_zone2 else 'OUT'} "
            f"| S={HV_ZONE_S[LANE_HV_ZONE_2]} | by {zone2_who}"
        )
        lines.append(
            f"HV_ZONE lane{LANE_HV_ZONE_3}: {'IN ' if hv_in_zone3 else 'OUT'} "
            f"| S={HV_ZONE_S[LANE_HV_ZONE_3]} | by {zone3_who}"
        )
        lines.append("---")

        for vid in CAV_IDS:
            in_zone, cav_abs_d, cav_mode = self._cav_in_own_zone(vid)

            # CAV zone 밖이면 무조건 FREE
            if not in_zone:
                gate_speed[vid] = FREE_SPEED
                lines.append(f"CAV{vid} | CAV_ZONE:OFF(abs_d={cav_abs_d:.2f},{cav_mode}) -> FREE")
                continue

            # CAV zone 안이면: HV 트리거 존 IN이면 FREE, 아니면 정지
            trig_lane = HV_TRIGGER_ZONE_LANE[vid]
            hv_ok = hv_in_zone2 if trig_lane == LANE_HV_ZONE_2 else hv_in_zone3

            gate_speed[vid] = FREE_SPEED if hv_ok else MIN_SPEED
            lines.append(
                f"CAV{vid} | CAV_ZONE:ON (abs_d={cav_abs_d:.2f},{cav_mode}) | "
                f"TRIG lane{trig_lane}={'IN' if hv_ok else 'OUT'} -> {gate_speed[vid]:.2f}"
            )

        lines.append("---")

        for vid in CAV_IDS:
            self._publish_gate_speed(vid, gate_speed[vid])
            lines.append(f"/CAV_{vid:02d}_gate_speed = {gate_speed[vid]:.2f}")

        lines.append("===")
        self.get_logger().info("\n" + "\n".join(lines))


def main():
    rclpy.init()
    node = HVTailgateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

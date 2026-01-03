#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import json
import os
import math
from geometry_msgs.msg import Accel, PoseStamped

# ===============================================================
# [1] 차량 및 시나리오 설정
# ===============================================================
VEHICLES = [
    {"id": 1, "path_file": "path_01.json", "pose_topic": "/CAV_01", "accel_topic": "/CAV_01_accel"},
    {"id": 2, "path_file": "path_02.json", "pose_topic": "/CAV_02", "accel_topic": "/CAV_02_accel"},
]

# 분석된 사지 교차로 중심 좌표 및 제어 파라미터
CRITICAL_ZONE = {"x": -2.5, "y": 0.0, "radius": 1.2} 
TARGET_VELOCITY = 0.48
REDUCED_VELOCITY = 0.2

# PID 및 주행 파라미터
LOOK_AHEAD_DISTANCE = 0.23
Kp, Ki, Kd, K_cte = 4.0, 0.05, 1.7, 6.0

class MultiVehicleDriver(Node):
    def __init__(self):
        super().__init__("multi_vehicle_driver")
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # 교차로 점유 상태 관리 (먼저 들어온 차량 ID)
        self.zone_occupant = None 

        self.vehicles = {}
        for v in VEHICLES:
            vid = v["id"]
            self.vehicles[vid] = {
                "cfg": v,
                "path_x": [], "path_y": [],
                "x": 0.0, "y": 0.0, "yaw": 0.0,
                "pose_ok": False,
                "prev_err": 0.0, "int_err": 0.0,
                "log_cnt": 0,
            }
            self._load_path_for_vehicle(vid)
            self.create_subscription(PoseStamped, v["pose_topic"], 
                                     lambda msg, _vid=vid: self._pose_cb(msg, _vid), qos_profile)
            self.vehicles[vid]["pub"] = self.create_publisher(Accel, v["accel_topic"], 10)

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self._control_loop)

    def _load_path_for_vehicle(self, vid: int):
        path_file = self.vehicles[vid]["cfg"]["path_file"]
        if not os.path.exists(path_file): return
        with open(path_file, "r") as f:
            data = json.load(f)
        self.vehicles[vid]["path_x"] = data.get("X") or data.get("x") or []
        self.vehicles[vid]["path_y"] = data.get("Y") or data.get("y") or []

    def _pose_cb(self, msg: PoseStamped, vid: int):
        st = self.vehicles[vid]
        st["pose_ok"], st["x"], st["y"] = True, msg.pose.position.x, msg.pose.position.y
        q = msg.pose.orientation
        st["yaw"] = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def _control_loop(self):
        # --- 1단계: 교차로 점유권 판단 ---
        for vid, st in self.vehicles.items():
            if not st["pose_ok"]: continue
            
            dist = math.hypot(st["x"] - CRITICAL_ZONE["x"], st["y"] - CRITICAL_ZONE["y"])
            
            if dist < CRITICAL_ZONE["radius"]:
                if self.zone_occupant is None:
                    self.zone_occupant = vid
                    print(f"🚩 [CAV_{vid}] 교차로 진입 및 점유 성공")
            elif self.zone_occupant == vid and dist > CRITICAL_ZONE["radius"] + 0.3:
                self.zone_occupant = None
                print(f"✅ [CAV_{vid}] 교차로 통과: 점유 해제")

        # --- 2단계: 결정된 상태에 따라 차량 제어 명령 생성 ---
        for vid, st in self.vehicles.items():
            if not st["pose_ok"] or len(st["path_x"]) == 0: continue

            dist = math.hypot(st["x"] - CRITICAL_ZONE["x"], st["y"] - CRITICAL_ZONE["y"])
            in_zone = dist < CRITICAL_ZONE["radius"]

            # 조향 계산
            steer, min_dist, _, _ = self._compute_steer_for_vehicle(st)
            
            # 속도 결정: 내가 구역 내에 있는데 점유자가 아니라면 양보(감속)
            if in_zone and self.zone_occupant != vid:
                current_vel = REDUCED_VELOCITY
                if st["log_cnt"] % 10 == 0:
                    print(f"⚠️ [CAV_{vid}] 충돌 상황(시나리오) 대기 중: {current_vel}m/s")
            else:
                current_vel = TARGET_VELOCITY

            # 명령 발행
            cmd = Accel()
            cmd.linear.x, cmd.angular.z = current_vel, steer
            st["pub"].publish(cmd)
            st["log_cnt"] += 1

    def _compute_steer_for_vehicle(self, st: dict):
        px, py, cx, cy, cyaw = st["path_x"], st["path_y"], st["x"], st["y"], st["yaw"]
        min_dist = float("inf")
        cur_idx = 0
        for i in range(len(px)):
            d = math.hypot(px[i] - cx, py[i] - cy)
            if d < min_dist: min_dist, cur_idx = d, i

        t_idx = cur_idx
        for i in range(cur_idx, len(px)):
            if math.hypot(px[i] - cx, py[i] - cy) >= LOOK_AHEAD_DISTANCE:
                t_idx = i
                break

        tx, ty = px[t_idx], py[t_idx]
        err = math.atan2(ty - cy, tx - cx) - cyaw
        while err > math.pi: err -= 2.0 * math.pi
        while err < -math.pi: err += 2.0 * math.pi

        st["int_err"] = max(-1.0, min(1.0, st["int_err"] + err * self.dt))
        p, i, d = Kp * err, Ki * st["int_err"], Kd * (err - st["prev_err"]) / self.dt
        cte = (min_dist * K_cte) if err > 0 else -(min_dist * K_cte)
        st["prev_err"] = err
        return max(min(p + i + d + cte, 1.0), -1.0), min_dist, err, cur_idx

def main(args=None):
    rclpy.init(args=args); node = MultiVehicleDriver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()

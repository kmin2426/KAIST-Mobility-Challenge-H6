#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import json
import os
import math
from geometry_msgs.msg import Accel, PoseStamped

# ===============================================================
# [1] 설정 및 제어 파라미터
# ===============================================================
VEHICLES = [
    {"id": 1, "path_file": "path_01.json", "pose_topic": "/CAV_01", "accel_topic": "/CAV_01_accel"},
    {"id": 2, "path_file": "path_02.json", "pose_topic": "/CAV_02", "accel_topic": "/CAV_02_accel"},
]

# 교차로 및 안전 거리 설정
CRITICAL_ZONE = {"x": -2.5, "y": 0.0, "radius": 1.4} 
COLLISION_THRESHOLD = 0.8  # 감속을 트리거할 차간 거리

TARGET_VELOCITY = 0.48
APPROACH_VELOCITY = 0.2    # 교차로 내 우선권 없을 때 기본 서행
MIN_MOVE_VELOCITY = 0.1    # 근접 시 최소 이동 속도 (정지 방지)

# 조향(PID) 파라미터
LOOK_AHEAD_DISTANCE = 0.23
Kp, Ki, Kd, K_cte = 4.0, 0.05, 1.7, 6.0

class MultiVehicleDriver(Node):
    def __init__(self):
        super().__init__("multi_vehicle_driver")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.zone_occupant = None 
        self.vehicles = {}

        for v in VEHICLES:
            vid = v["id"]
            self.vehicles[vid] = {
                "cfg": v, "path_x": [], "path_y": [],
                "x": 0.0, "y": 0.0, "yaw": 0.0, "pose_ok": False,
                "prev_err": 0.0, "int_err": 0.0, "log_cnt": 0,
            }
            self._load_path_for_vehicle(vid)
            self.create_subscription(PoseStamped, v["pose_topic"], 
                                     lambda msg, _vid=vid: self._pose_cb(msg, _vid), qos)
            self.vehicles[vid]["pub"] = self.create_publisher(Accel, v["accel_topic"], 10)

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self._control_loop)

    def _load_path_for_vehicle(self, vid):
        path_file = self.vehicles[vid]["cfg"]["path_file"]
        if os.path.exists(path_file):
            with open(path_file, "r") as f:
                data = json.load(f)
                self.vehicles[vid]["path_x"] = data.get("X") or data.get("x") or []
                self.vehicles[vid]["path_y"] = data.get("Y") or data.get("y") or []

    def _pose_cb(self, msg, vid):
        st = self.vehicles[vid]
        st["pose_ok"], st["x"], st["y"] = True, msg.pose.position.x, msg.pose.position.y
        q = msg.pose.orientation
        st["yaw"] = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def _control_loop(self):
        if not all(v["pose_ok"] for v in self.vehicles.values()): return

        # 1. 차량 간 실시간 데이터 계산
        v1, v2 = self.vehicles[1], self.vehicles[2]
        dist_between = math.hypot(v1["x"] - v2["x"], v1["y"] - v2["y"])
        
        # [핵심] 병행 주행 여부 판단 (Heading Dot Product)
        # cos_diff가 1에 가까우면 같은 방향, 0이면 직교, -1이면 정면 대향
        cos_diff = math.cos(v1["yaw"]) * math.cos(v2["yaw"]) + math.sin(v1["yaw"]) * math.sin(v2["yaw"])
        is_parallel = cos_diff > 0.8  # 약 36도 이내의 각도로 나란히 가는 경우

        # 2. 교차로 점유권 업데이트
        for vid, st in self.vehicles.items():
            dist_to_c = math.hypot(st["x"] - CRITICAL_ZONE["x"], st["y"] - CRITICAL_ZONE["y"])
            if dist_to_c < CRITICAL_ZONE["radius"]:
                if self.zone_occupant is None:
                    self.zone_occupant = vid
            elif self.zone_occupant == vid and dist_to_c > CRITICAL_ZONE["radius"] + 0.5:
                self.zone_occupant = None

        # 3. 개별 차량 속도 제어
        for vid, st in self.vehicles.items():
            if not st["path_x"]: continue

            current_vel = TARGET_VELOCITY
            dist_to_c = math.hypot(st["x"] - CRITICAL_ZONE["x"], st["y"] - CRITICAL_ZONE["y"])
            in_zone = dist_to_c < CRITICAL_ZONE["radius"]
            
            # [기본 교차로 제어]
            if in_zone and self.zone_occupant != vid:
                current_vel = APPROACH_VELOCITY

            # [거리 기반 제어: 병행 주행 시에는 무시]
            # 1. 두 차량이 나란히 가지 않고(평행이 아니고)
            # 2. 거리가 임계값보다 가깝다면
            if not is_parallel and dist_between < COLLISION_THRESHOLD:
                # 우선권이 있는 차가 아니라면 감속
                if self.zone_occupant is not None and vid != self.zone_occupant:
                    current_vel = MIN_MOVE_VELOCITY
                # 우선권이 없는 지역(교차로 밖)에서 마주칠 때 (Head-on 방지)
                elif self.zone_occupant is None and vid == 2:
                    current_vel = MIN_MOVE_VELOCITY

            # 조향 계산 및 명령 발행
            steer, _, _, _ = self._compute_steer(st)
            cmd = Accel()
            cmd.linear.x, cmd.angular.z = current_vel, steer
            st["pub"].publish(cmd)
            st["log_cnt"] += 1

    def _compute_steer(self, st):
        px, py, cx, cy, cyaw = st["path_x"], st["path_y"], st["x"], st["y"], st["yaw"]
        min_dist, cur_idx = min((math.hypot(px[i]-cx, py[i]-cy), i) for i in range(len(px)))
        t_idx = cur_idx
        for i in range(cur_idx, len(px)):
            if math.hypot(px[i]-cx, py[i]-cy) >= LOOK_AHEAD_DISTANCE:
                t_idx = i; break
        err = math.atan2(py[t_idx]-cy, px[t_idx]-cx) - cyaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi
        st["int_err"] = max(-1.0, min(1.0, st["int_err"] + err * 0.05))
        p, i, d = Kp * err, Ki * st["int_err"], Kd * (err - st["prev_err"]) / 0.05
        cte = (min_dist * K_cte) if err > 0 else -(min_dist * K_cte)
        st["prev_err"] = err
        return max(min(p + i + d + cte, 1.0), -1.0), min_dist, err, cur_idx

def main():
    rclpy.init(); node = MultiVehicleDriver()
    try: rclpy.spin(node)
    except: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()

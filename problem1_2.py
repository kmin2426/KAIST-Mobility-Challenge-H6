#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import json
import os
import math
from geometry_msgs.msg import Accel, PoseStamped

# ===============================================================
# [1] 차량별 설정 (1번 + 2번 동시 제어)
# ===============================================================
VEHICLES = [
    {
        "id": 1,
        "path_file": "path_01.json",
        "pose_topic": "/CAV_01",
        "accel_topic": "/CAV_01_accel",
    },
    {
        "id": 2,
        "path_file": "path_02.json",
        "pose_topic": "/CAV_02",
        "accel_topic": "/CAV_02_accel",
    },
]

print("\n🟢 [멀티 차량 제어 모드] 시작 (CAV_01 + CAV_02)")
for v in VEHICLES:
    print(f"   - 차량 {v['id']}: path={v['path_file']} | pose={v['pose_topic']} | cmd={v['accel_topic']}")

# ===============================================================
# [2] 공통 튜닝 파라미터 (너가 쓰던 값 그대로)
# ===============================================================
TARGET_VELOCITY = 0.48
LOOK_AHEAD_DISTANCE = 0.23

Kp = 4.0
Ki = 0.05
Kd = 1.7
K_cte = 6.0


class MultiVehicleDriver(Node):
    def __init__(self):
        super().__init__("multi_vehicle_driver")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 차량별 상태 저장소
        self.vehicles = {}
        for v in VEHICLES:
            vid = v["id"]

            # 차량별 runtime 상태
            self.vehicles[vid] = {
                "cfg": v,

                # path
                "path_x": [],
                "path_y": [],

                # pose
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "pose_ok": False,

                # PID states
                "prev_err": 0.0,
                "int_err": 0.0,

                # debug
                "log_cnt": 0,
            }

            # path load
            self._load_path_for_vehicle(vid)

            # pub/sub
            self.create_subscription(
                PoseStamped,
                v["pose_topic"],
                lambda msg, _vid=vid: self._pose_cb(msg, _vid),
                qos_profile,
            )

            self.vehicles[vid]["pub"] = self.create_publisher(
                Accel, v["accel_topic"], 10
            )

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self._control_loop)

    # ---------------------------
    # Path Loader
    # ---------------------------
    def _load_path_for_vehicle(self, vid: int):
        path_file = self.vehicles[vid]["cfg"]["path_file"]

        if not os.path.exists(path_file):
            self.get_logger().error(f"❌ [CAV_{vid:02d}] 경로 파일 없음: {path_file}")
            return

        try:
            with open(path_file, "r") as f:
                data = json.load(f)

            path_x = data.get("X") or data.get("x") or []
            path_y = data.get("Y") or data.get("y") or []

            if len(path_x) == 0 or len(path_x) != len(path_y):
                self.get_logger().error(
                    f"❌ [CAV_{vid:02d}] 경로 데이터 이상: X({len(path_x)}), Y({len(path_y)})"
                )
                return

            self.vehicles[vid]["path_x"] = path_x
            self.vehicles[vid]["path_y"] = path_y
            self.get_logger().info(f"✅ [CAV_{vid:02d}] 경로 로드 완료: {len(path_x)} points")

        except Exception as e:
            self.get_logger().error(f"❌ [CAV_{vid:02d}] 경로 로드 오류: {e}")

    # ---------------------------
    # Pose Callback
    # ---------------------------
    def _pose_cb(self, msg: PoseStamped, vid: int):
        st = self.vehicles[vid]

        if not st["pose_ok"]:
            self.get_logger().info(f"✨ [CAV_{vid:02d}] 시뮬레이터 연결 성공! ({st['cfg']['pose_topic']})")
        st["pose_ok"] = True

        st["x"] = msg.pose.position.x
        st["y"] = msg.pose.position.y

        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        st["yaw"] = math.atan2(siny_cosp, cosy_cosp)

    # ---------------------------
    # Control (timer)
    # ---------------------------
    def _control_loop(self):
        for vid, st in self.vehicles.items():
            if not st["pose_ok"]:
                continue
            if len(st["path_x"]) == 0:
                continue

            steer, min_dist, yaw_err, cur_idx = self._compute_steer_for_vehicle(st)

            # publish
            cmd = Accel()
            cmd.linear.x = TARGET_VELOCITY
            cmd.angular.z = steer
            st["pub"].publish(cmd)

            # debug print
            st["log_cnt"] += 1
            if st["log_cnt"] % 5 == 0:
                print(
                    f"[CAV_{vid:02d}] [{cur_idx}] "
                    f"Err(거리):{min_dist:.3f}m | "
                    f"YawErr(각도):{math.degrees(yaw_err):.1f}° | "
                    f"Steer(명령):{steer:.3f}"
                )

    def _compute_steer_for_vehicle(self, st: dict):
        px = st["path_x"]
        py = st["path_y"]
        cx = st["x"]
        cy = st["y"]
        cyaw = st["yaw"]

        # 1) nearest point
        min_dist = float("inf")
        current_idx = 0
        for i in range(len(px)):
            d = math.hypot(px[i] - cx, py[i] - cy)
            if d < min_dist:
                min_dist = d
                current_idx = i

        # 2) look-ahead point
        target_idx = current_idx
        for i in range(current_idx, len(px)):
            d = math.hypot(px[i] - cx, py[i] - cy)
            if d >= LOOK_AHEAD_DISTANCE:
                target_idx = i
                break

        tx, ty = px[target_idx], py[target_idx]

        # 3) desired yaw & yaw error
        desired_yaw = math.atan2(ty - cy, tx - cx)
        yaw_err = desired_yaw - cyaw

        # normalize to [-pi, pi]
        while yaw_err > math.pi:
            yaw_err -= 2.0 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2.0 * math.pi

        # PID on yaw_err
        st["int_err"] += yaw_err * self.dt
        st["int_err"] = max(-1.0, min(1.0, st["int_err"]))

        p = Kp * yaw_err
        i = Ki * st["int_err"]
        d = Kd * (yaw_err - st["prev_err"]) / self.dt

        # CTE boost (너가 쓰던 휴리스틱 그대로)
        cte = min_dist * K_cte
        if yaw_err < 0:
            cte = -cte

        steer = p + i + d + cte
        st["prev_err"] = yaw_err

        # clamp
        steer = max(min(steer, 1.0), -1.0)
        return steer, min_dist, yaw_err, current_idx


def main(args=None):
    rclpy.init(args=args)
    node = MultiVehicleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

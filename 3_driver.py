#!/usr/bin/env python3
import os
import math
import json

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Accel

# ============================================================
# [IO helpers]
# ============================================================
def load_path_points(json_file: str):
    if not os.path.exists(json_file):
        return []
    with open(json_file, "r") as f:
        data = json.load(f)

    pts = []
    if isinstance(data, dict):
        xs = data.get("x") or data.get("X")
        ys = data.get("y") or data.get("Y")
        if xs and ys:
            for x, y in zip(xs, ys):
                pts.append((float(x), float(y)))
    return pts


# ============================================================
# [Global Settings]  (네가 보낸 값 그대로 유지)
# ============================================================
TARGET_VELOCITY = 0.48
LOOK_AHEAD_DISTANCE = 0.37
WHEELBASE = 0.211
DIST_CENTER_TO_REAR = WHEELBASE / 2.0

Kp, Ki, Kd = 6.0, 0.055, 1.0
K_cte = 5.0


# ============================================================
# [Driver] (네가 보낸 Driver 구조 그대로 + vehicle_id만 확장)
# ============================================================
class ZonePriorityDriver(Node):
    def __init__(self, vehicle_id: int):
        super().__init__(f'zone_driver_{vehicle_id}')
        self.vehicle_id = int(vehicle_id)

        # ✅ 기존 규칙 그대로 확장
        self.MY_PATH_FILE = f'path3_{self.vehicle_id}.json'
        self.MY_TOPIC = f'/CAV_{self.vehicle_id:02d}'

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.accel_publisher = self.create_publisher(Accel, f'{self.MY_TOPIC}_accel', 10)
        self.create_subscription(PoseStamped, self.MY_TOPIC, self.pose_callback, qos)

        self.path_points = load_path_points(self.MY_PATH_FILE)
        self.path_x = [p[0] for p in self.path_points]
        self.path_y = [p[1] for p in self.path_points]

        if not self.path_x:
            self.get_logger().error(f"[Car{self.vehicle_id}] Path file missing/empty: {self.MY_PATH_FILE}")
        else:
            self.get_logger().info(f"[Car{self.vehicle_id}] Path loaded: {self.MY_PATH_FILE} ({len(self.path_x)} pts)")

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.is_pose_received = False

        self.prev_error = 0.0
        self.integral_error = 0.0

        self.last_time = self.get_clock().now()
        self.create_timer(0.05, self.drive_callback)
        self.log_counter = 0

    def pose_callback(self, msg):
        self.is_pose_received = True

        # ✅ 네 코드 그대로: orientation.z가 yaw
        raw_yaw = msg.pose.orientation.z

        # ✅ 중심 -> 뒷바퀴 좌표 보정
        self.current_yaw = raw_yaw
        self.current_x = msg.pose.position.x - (DIST_CENTER_TO_REAR * math.cos(self.current_yaw))
        self.current_y = msg.pose.position.y - (DIST_CENTER_TO_REAR * math.sin(self.current_yaw))

    def drive_callback(self):
        if not self.is_pose_received or not self.path_x:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0.001:
            return
        if dt > 0.1:
            dt = 0.1

        # 1) closest point
        min_dist = float('inf')
        current_idx = 0
        for i, (px, py) in enumerate(zip(self.path_x, self.path_y)):
            d = math.hypot(px - self.current_x, py - self.current_y)
            if d < min_dist:
                min_dist = d
                current_idx = i

        # 2) look-ahead target
        target_idx = current_idx
        for i in range(current_idx, len(self.path_x)):
            if math.hypot(self.path_x[i] - self.current_x, self.path_y[i] - self.current_y) >= LOOK_AHEAD_DISTANCE:
                target_idx = i
                break

        tx, ty = self.path_x[target_idx], self.path_y[target_idx]

        # 3) speed (driver only)
        final_velocity = TARGET_VELOCITY

        # 4) steering (PID + CTE correction) — 네 코드 그대로
        desired_yaw = math.atan2(ty - self.current_y, tx - self.current_x)
        yaw_err = desired_yaw - self.current_yaw
        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi

        self.integral_error = max(-1.0, min(1.0, self.integral_error + yaw_err * dt))
        p = Kp * yaw_err
        i = Ki * self.integral_error
        d = Kd * (yaw_err - self.prev_error) / dt
        cte = min_dist * K_cte * (-1 if yaw_err < 0 else 1)

        final_steering = max(-1.0, min(1.0, p + i + d + cte))
        self.prev_error = yaw_err

        # 5) publish
        cmd = Accel()
        cmd.linear.x = float(final_velocity)
        cmd.angular.z = float(final_steering)
        self.accel_publisher.publish(cmd)

        

# ============================================================
# main: 한 번 실행하면 1~4 다 실행
# ============================================================
def main():
    rclpy.init(args=None)

    drivers = [
        ZonePriorityDriver(1),
        ZonePriorityDriver(2),
        ZonePriorityDriver(3),
        ZonePriorityDriver(4),
    ]

    ex = MultiThreadedExecutor(num_threads=6)
    for n in drivers:
        ex.add_node(n)

    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        for n in drivers:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


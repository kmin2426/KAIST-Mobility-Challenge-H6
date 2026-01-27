#!/usr/bin/env python3
import os
import json
import math
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist

PATH = "path3_2.json"
NUM = 9 #CAV Number
SPEED = 0.5 #CAV Fake Speed

def load_path_points(json_file: str):
    if not os.path.exists(json_file):
        raise FileNotFoundError(f"Path file not found: {json_file}")

    with open(json_file, "r") as f:
        data = json.load(f)

    xs = data.get("x") or data.get("X")
    ys = data.get("y") or data.get("Y")
    if not xs or not ys or len(xs) != len(ys):
        raise ValueError(f"Invalid path json format: {json_file}")

    pts = [(float(x), float(y)) for x, y in zip(xs, ys)]
    if len(pts) < 2:
        raise ValueError("Path must have >= 2 points")
    return pts


class FakePosePub(Node):
    """
    Publish PoseStamped to /CAV_XX following path points with linear interpolation.
    - orientation.z is used as yaw (your system convention).
    - speed is driven by incoming /CAV_XX/cmd_vel, but you can force constant 1.0 m/s.
    """
    def __init__(
        self,
        topic_pose: str,
        path_pts,
        rate_hz: float,
        loop: bool,
        force_speed_mps: float = 1.0,
        cmd_timeout_s: float = 0.5,
        use_cmd_vel: bool = False,
    ):
        super().__init__("fake_pose_pub_path")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(PoseStamped, topic_pose, qos)
        self.topic_pose = topic_pose
        self.cmd_topic = f"{topic_pose}/cmd_vel"

        self.pts = path_pts
        self.loop = loop

        self.dt = 1.0 / float(rate_hz)

        # Mode:
        # - use_cmd_vel=False  -> always publish with force_speed_mps (default 1.0 m/s)
        # - use_cmd_vel=True   -> follow cmd_vel.linear.x, fallback to force_speed_mps on timeout
        self.use_cmd_vel = bool(use_cmd_vel)
        self.force_speed_mps = float(force_speed_mps)
        self.cmd_timeout_s = float(cmd_timeout_s)

        # latest cmd_vel
        self.curr_v = self.force_speed_mps
        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(
            Twist,
            self.cmd_topic,
            self._cmd_cb,
            10
        )

        # segment state
        self.i = 0      # segment start index
        self.s = 0.0    # traveled along current segment [m]

        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(f"[FakePose] pose_topic={self.topic_pose}")
        self.get_logger().info(f"[FakePose] cmd_topic ={self.cmd_topic}")
        self.get_logger().info(f"[FakePose] rate={rate_hz}Hz dt={self.dt:.4f}s loop={self.loop}")
        self.get_logger().info(f"[FakePose] pts={len(self.pts)}")
        self.get_logger().info(f"[FakePose] mode={'cmd_vel' if self.use_cmd_vel else 'FORCED_CONST'}")
        self.get_logger().info(f"[FakePose] force_speed={self.force_speed_mps:.2f} m/s (default=1.0)")

    def _cmd_cb(self, msg: Twist):
        self.curr_v = float(msg.linear.x)
        self.last_cmd_time = self.get_clock().now()

    def _get_speed(self) -> float:
        if not self.use_cmd_vel:
            return self.force_speed_mps

        # follow cmd_vel; if stale, fallback to force_speed_mps
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout_s:
            return self.force_speed_mps
        return self.curr_v

    def tick(self):
        # end handling
        if self.i >= len(self.pts) - 1:
            if self.loop:
                self.i = 0
                self.s = 0.0
            else:
                x, y = self.pts[-1]
                yaw = 0.0
                self._pub_pose(x, y, yaw)
                self.get_logger().info("[FakePose] Reached end of path. Stopping publisher.")
                self.timer.cancel()
                return

        x0, y0 = self.pts[self.i]
        x1, y1 = self.pts[self.i + 1]

        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)

        # skip zero-length segment
        if seg_len < 1e-6:
            self.i += 1
            self.s = 0.0
            return

        # advance along segment
        v = max(0.0, float(self._get_speed()))
        self.s += v * self.dt

        # carry over to next segment(s)
        while self.s > seg_len and self.i < len(self.pts) - 2:
            self.s -= seg_len
            self.i += 1
            x0, y0 = self.pts[self.i]
            x1, y1 = self.pts[self.i + 1]
            dx = x1 - x0
            dy = y1 - y0
            seg_len = math.hypot(dx, dy)
            if seg_len < 1e-6:
                seg_len = 1e-6

        # interpolation ratio
        t = max(0.0, min(1.0, self.s / seg_len))

        x = x0 + t * dx
        y = y0 + t * dy
        yaw = math.atan2(dy, dx)  # path tangent yaw

        self._pub_pose(x, y, yaw)

    def _pub_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = float(yaw)  # IMPORTANT: your system uses z as yaw
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, default=None, help="ROS_DOMAIN_ID (optional; recommend export outside)")
    parser.add_argument("--cav", type=int, default=NUM, help="CAV topic number (e.g., 9 -> /CAV_09)")
    parser.add_argument("--path", type=str, default=None, help="path json file, e.g. path/path3_2.json")
    parser.add_argument("--rate", type=float, default=50.0, help="publish rate Hz (e.g., 50 for 0.02s)")
    parser.add_argument("--loop", action="store_true", help="loop path forever")

    # You asked: "1.0 m/s 로 발행"
    # Default: always 1.0 m/s (forced constant)
    parser.add_argument("--speed", type=float, default=SPEED, help="forced speed m/s (default 1.0)")
    parser.add_argument("--use_cmd_vel", action="store_true", help="if set, follow /CAV_XX/cmd_vel speed (fallback to --speed)")
    parser.add_argument("--cmd_timeout", type=float, default=0.5, help="cmd_vel timeout seconds (fallback to --speed)")

    args = parser.parse_args()

    if args.domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain)

    rclpy.init()

    base_dir = os.path.dirname(os.path.abspath(__file__))
    default_path = os.path.join(base_dir, "path", PATH)
    path_file = args.path if args.path else default_path

    pts = load_path_points(path_file)

    pose_topic = f"/CAV_{int(args.cav):02d}"
    node = FakePosePub(
        topic_pose=pose_topic,
        path_pts=pts,
        rate_hz=args.rate,
        loop=args.loop,
        force_speed_mps=args.speed,     # default 1.0
        cmd_timeout_s=args.cmd_timeout,
        use_cmd_vel=args.use_cmd_vel,   # default False -> forced 1.0
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import math
import json
import os
from enum import IntEnum
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Accel
from std_msgs.msg import Int32


# Path files for each lane
LANE_PATH_FILES = {
    0: '/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/path_2_1.json',
    1: '/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/path_2_2.json',
    2: '/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/path_2_3.json',
}

ALLOWED_LANES = [0, 1, 2]

# Controller Parameters
SPEED_GO = 0.8
SPEED_SLOW = 0.3

# Normal (KEEP/PREPARE)
N_LOOKAHEAD_MIN = 0.50
N_LOOKAHEAD_MAX = 0.70
N_STEER_GAIN = 1.0
N_CTE_GAIN = 0.8
N_FILTER_ALPHA = 0.35
N_MAX_W = 1.2  # rad/s clamp

# Aggressive (LANE_CHANGE)
A_LOOKAHEAD = 0.35         # lookahead 줄이기 (급하게 꺾임)
A_STEER_GAIN = 1.6
A_CTE_GAIN = 1.2
A_FILTER_ALPHA = 0.90      # 필터 덜 누르기 (반응 빠르게)
A_MAX_W = 2.5              # rad/s clamp (더 크게 허용)


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def load_path_json(path_file: str) -> Tuple[List[float], List[float]]:
    with open(path_file, "r") as f:
        data = json.load(f)

    if isinstance(data, dict) and "X" in data and "Y" in data:
        return list(map(float, data["X"])), list(map(float, data["Y"]))

    raise ValueError(f"Unsupported path json format: {path_file}")


class Behavior(IntEnum):
    GO = 0
    STOP = 1
    SLOW = 2


class FSMState(IntEnum):
    KEEP_LANE = 0
    PREPARE_LC = 1
    LANE_CHANGE = 2


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")

        # topics
        self.declare_parameter("pose_topic", "/CAV_01")
        self.declare_parameter("cmd_topic", "/CAV_01_accel")
        self.declare_parameter("lane_topic", "/cav/target_lane")
        self.declare_parameter("behavior_topic", "/cav/behavior")
        self.declare_parameter("fsm_state_topic", "/cav/fsm_state")

        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        self.lane_topic = self.get_parameter("lane_topic").get_parameter_value().string_value
        self.behavior_topic = self.get_parameter("behavior_topic").get_parameter_value().string_value
        self.fsm_state_topic = self.get_parameter("fsm_state_topic").get_parameter_value().string_value

        # pose
        self.is_pose_received = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # decision inputs
        self.target_lane: Optional[int] = None
        self.behavior: int = int(Behavior.GO)
        self.fsm_state: int = int(FSMState.KEEP_LANE)

        # path
        self.path_x: List[float] = []
        self.path_y: List[float] = []

        # steering filter memory
        self.prev_angular_vel = 0.0

        # ROS
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(Int32, self.lane_topic, self.lane_cb, 10)
        self.create_subscription(Int32, self.behavior_topic, self.behavior_cb, 10)
        self.create_subscription(Int32, self.fsm_state_topic, self.fsm_state_cb, 10)
        self.cmd_pub = self.create_publisher(Accel, self.cmd_topic, 10)

        self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info(
            f"[controller] pose={self.pose_topic}, cmd={self.cmd_topic}\n"
            f"            lane={self.lane_topic}, behavior={self.behavior_topic}, fsm_state={self.fsm_state_topic}"
        )

    def pose_cb(self, msg: PoseStamped):
        self.current_x = float(msg.pose.position.x)
        self.current_y = float(msg.pose.position.y)
        self.current_yaw = msg.pose.orientation.z
        self.is_pose_received = True

    def lane_cb(self, msg: Int32):
        lane = int(msg.data)
        if lane not in ALLOWED_LANES:
            return
        if self.target_lane == lane and self.path_x:
            return

        self.target_lane = lane
        path_file = LANE_PATH_FILES.get(lane)
        if not path_file:
            self.get_logger().error(f"[controller] no path mapping for lane={lane}")
            return
        if not os.path.exists(path_file):
            self.get_logger().error(f"[controller] FILE NOT FOUND: {path_file}")
            self.path_x, self.path_y = [], []
            return

        try:
            px, py = load_path_json(path_file)
            if len(px) < 2 or len(px) != len(py):
                raise ValueError("invalid path length")
            self.path_x, self.path_y = px, py
            self.prev_angular_vel = 0.0  # lane 바뀔 때 반동 줄이기
            self.get_logger().info(f"[controller] target_lane={lane} loaded '{path_file}' N={len(px)}")
        except Exception as e:
            self.get_logger().error(f"[controller] failed to load '{path_file}': {repr(e)}")
            self.path_x, self.path_y = [], []

    def behavior_cb(self, msg: Int32):
        self.behavior = int(msg.data)

    def fsm_state_cb(self, msg: Int32):
        self.fsm_state = int(msg.data)

    def get_nearest_index(self) -> int:
        best_i = 0
        best_d2 = float("inf")
        cx, cy = self.current_x, self.current_y
        for i in range(len(self.path_x)):
            dx = self.path_x[i] - cx
            dy = self.path_y[i] - cy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def control_loop(self):
        if not self.is_pose_received:
            return
        if not self.path_x:
            return

        # STOP이면 완전 정지(조향도 0으로)
        if self.behavior == int(Behavior.STOP):
            cmd = Accel()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.prev_angular_vel = 0.0
            self.cmd_pub.publish(cmd)
            return

        # 속도 선택 (GO/SLOW 분리)
        if self.behavior == int(Behavior.SLOW):
            speed = SPEED_SLOW
        else:
            speed = SPEED_GO

        # 공격모드 조건
        aggressive = (self.fsm_state == int(FSMState.LANE_CHANGE))

        # params 선택
        if aggressive:
            lookahead = A_LOOKAHEAD
            steer_gain = A_STEER_GAIN
            cte_gain = A_CTE_GAIN
            alpha = A_FILTER_ALPHA
            max_w = A_MAX_W
        else:
            lookahead = min(max(N_LOOKAHEAD_MIN, speed * 0.5), N_LOOKAHEAD_MAX)
            steer_gain = N_STEER_GAIN
            cte_gain = N_CTE_GAIN
            alpha = N_FILTER_ALPHA
            max_w = N_MAX_W

        nearest_idx = self.get_nearest_index()

        # lookahead target index
        target_idx = nearest_idx
        while target_idx < len(self.path_x) - 1:
            dx = self.path_x[target_idx] - self.current_x
            dy = self.path_y[target_idx] - self.current_y
            if math.hypot(dx, dy) > lookahead:
                break
            target_idx += 1

        # target point
        tx = self.path_x[target_idx]
        ty = self.path_y[target_idx]
        dx = tx - self.current_x
        dy = ty - self.current_y

        # body frame lateral at lookahead
        local_y = math.sin(-self.current_yaw) * dx + math.cos(-self.current_yaw) * dy

        # curvature
        curvature = 2.0 * local_y / (lookahead ** 2)

        # CTE at nearest
        near_dx = self.path_x[nearest_idx] - self.current_x
        near_dy = self.path_y[nearest_idx] - self.current_y
        cte_y = math.sin(-self.current_yaw) * near_dx + math.cos(-self.current_yaw) * near_dy

        # steering command
        pp_steer = curvature * steer_gain
        cte_steer = cte_y * cte_gain
        final_steer_cmd = pp_steer + cte_steer

        # angular velocity = v * steer_cmd
        # (공격모드에서는 기존처럼 조향이 죽지 않게 steer_speed를 GO로 유지)
        steer_speed = SPEED_GO if aggressive else speed
        raw_w = steer_speed * final_steer_cmd

        # filter
        filt_w = alpha * raw_w + (1.0 - alpha) * self.prev_angular_vel
        self.prev_angular_vel = filt_w

        # clamp (공격모드에서 한도↑)
        filt_w = clamp(filt_w, -max_w, max_w)

        # publish
        cmd = Accel()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(filt_w)
        self.cmd_pub.publish(cmd)

        # Debug
        self.get_logger().info(
            f'[CTRL] lane={self.target_lane} '
            f'behavior={self.behavior} '
            f'fsm={self.fsm_state} '
            f'speed={speed:.2f} '
            f'w={filt_w:.2f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

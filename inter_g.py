#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Accel
import math


class IntersectionGuardian(Node):
    def __init__(self):
        super().__init__('intersection_guardian')

        # ================== 파라미터 ==================
        self.CENTER = (-2.5351, 0.0)

        self.RADIUS = 1.1333          # 관제 진입 반경
        self.CORE_RADIUS = 0.85      # 중심부 선제 감속 영역
        self.EXIT_RADIUS = 0.6        # 관제 해제 판단 반경

        self.V_NOM = 0.48

        self.D_SAFE = 1.5             # 감속 비례 기준
        self.D_CLEAR = 1.2            # 근접 충돌 판단 거리

        self.TTC_THRESHOLD = 0.1
        self.HYSTERESIS_N = 10        # 약 0.5초

        # ================== 상태 변수 ==================
        self.poses = {1: None, 2: None}
        self.last_poses = {1: None, 2: None}
        self.prev_dist_c = {1: 0.0, 2: 0.0}
        self.current_speeds = {1: self.V_NOM, 2: self.V_NOM}

        self.leader_id = None
        self.exit_counter = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(PoseStamped, '/CAV_01',
                                 lambda msg: self.cb_pose(1, msg), qos)
        self.create_subscription(PoseStamped, '/CAV_02',
                                 lambda msg: self.cb_pose(2, msg), qos)

        self.pub_v1 = self.create_publisher(Accel, '/CAV_01_target_speed', 10)
        self.pub_v2 = self.create_publisher(Accel, '/CAV_02_target_speed', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

    # ================== Pose 콜백 ==================
    def cb_pose(self, cav_id, msg):
        curr_p = (msg.pose.position.x, msg.pose.position.y)

        if self.last_poses[cav_id] is not None:
            prev_p = self.last_poses[cav_id]
            dist = math.hypot(curr_p[0] - prev_p[0], curr_p[1] - prev_p[1])
            v_est = dist / 0.05
            self.current_speeds[cav_id] = 0.3 * v_est + 0.7 * self.current_speeds[cav_id]

        self.last_poses[cav_id] = curr_p
        self.poses[cav_id] = curr_p

    # ================== 메인 제어 루프 ==================
    def control_loop(self):
        if not all(self.poses.values()):
            return

        p1, p2 = self.poses[1], self.poses[2]

        dist_c1 = math.hypot(p1[0] - self.CENTER[0], p1[1] - self.CENTER[1])
        dist_c2 = math.hypot(p2[0] - self.CENTER[0], p2[1] - self.CENTER[1])
        dist_between = math.hypot(p1[0] - p2[0], p1[1] - p2[1])

        target_v1 = self.V_NOM
        target_v2 = self.V_NOM

        # ================== 우선권 결정 ==================
        if self.leader_id is None:
            if dist_c1 < self.RADIUS or dist_c2 < self.RADIUS:
                v1 = max(0.01, self.current_speeds[1])
                v2 = max(0.01, self.current_speeds[2])

                ttc1 = dist_c1 / v1
                ttc2 = dist_c2 / v2

                if abs(ttc1 - ttc2) < self.TTC_THRESHOLD:
                    self.leader_id = 1
                else:
                    self.leader_id = 1 if ttc1 < ttc2 else 2

                self.exit_counter = 0

        # ================== 관제 로직 ==================
        if self.leader_id is not None:
            d_l = dist_c1 if self.leader_id == 1 else dist_c2
            prev_d_l = self.prev_dist_c[self.leader_id]

            # ---- 관제 해제 판단 ----
            if d_l > prev_d_l and d_l > self.EXIT_RADIUS:
                self.exit_counter += 1
            else:
                self.exit_counter = 0

            # ---- 감속 활성 조건 ----
            collision_active = (
                dist_c1 < self.RADIUS and
                dist_c2 < self.RADIUS and
                dist_between < self.D_CLEAR
            )

            core_active = (
                dist_c1 < self.CORE_RADIUS and
                dist_c2 < self.CORE_RADIUS
            )

            slow_active = collision_active or core_active

            # ---- 감속 적용 ----
            if self.exit_counter < self.HYSTERESIS_N and slow_active:
                ratio = max(0.2, min(1.0, dist_between / self.D_SAFE))
                v_slow = self.V_NOM * ratio

                if self.leader_id == 1:
                    target_v2 = v_slow
                else:
                    target_v1 = v_slow

            # ---- 관제 완전 해제 ----
            if self.exit_counter >= self.HYSTERESIS_N:
                self.leader_id = None
                self.exit_counter = 0

        self.prev_dist_c[1] = dist_c1
        self.prev_dist_c[2] = dist_c2

        self.send_speed(1, target_v1)
        self.send_speed(2, target_v2)

    def send_speed(self, cav_id, speed):
        msg = Accel()
        msg.linear.x = float(speed)
        if cav_id == 1:
            self.pub_v1.publish(msg)
        else:
            self.pub_v2.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(IntersectionGuardian())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

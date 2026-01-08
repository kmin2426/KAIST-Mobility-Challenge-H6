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
# [Global Settings]
# ============================================================
TARGET_VELOCITY = 0.48
LOOK_AHEAD_DISTANCE = 0.37
WHEELBASE = 0.211
DIST_CENTER_TO_REAR = WHEELBASE / 2.0

Kp, Ki, Kd = 6.0, 0.055, 1.0
K_cte = 5.0


# ============================================================
# [Driver] publish to *_accel_raw ONLY
# ============================================================
class ZonePriorityDriver(Node):
    def __init__(self, vehicle_id: int):
        super().__init__(f'zone_driver_{vehicle_id}')
        self.vehicle_id = int(vehicle_id)

        self.MY_PATH_FILE = f'path3_{self.vehicle_id}.json'
        self.MY_TOPIC = f'/CAV_{self.vehicle_id:02d}'

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ✅ raw only
        self.accel_raw_pub = self.create_publisher(Accel, f'{self.MY_TOPIC}_accel_raw', 10)
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

        # 명세: orientation.z가 yaw
        raw_yaw = msg.pose.orientation.z

        # center -> rear 보정
        self.current_yaw = raw_yaw
        self.current_x = msg.pose.position.x - (DIST_CENTER_TO_REAR * math.cos(self.current_yaw))
        self.current_y = msg.pose.position.y - (DIST_CENTER_TO_REAR * math.sin(self.current_yaw))

    def drive_callback(self):
        if not self.is_pose_received or not self.path_x:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.001:
            return
        if dt > 0.1:
            dt = 0.1

        # closest point
        min_dist = float('inf')
        current_idx = 0
        for i, (px, py) in enumerate(zip(self.path_x, self.path_y)):
            d = math.hypot(px - self.current_x, py - self.current_y)
            if d < min_dist:
                min_dist = d
                current_idx = i

        # look-ahead target
        target_idx = current_idx
        for i in range(current_idx, len(self.path_x)):
            if math.hypot(self.path_x[i] - self.current_x, self.path_y[i] - self.current_y) >= LOOK_AHEAD_DISTANCE:
                target_idx = i
                break

        tx, ty = self.path_x[target_idx], self.path_y[target_idx]

        # steering (PID + CTE correction)
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

        steer = max(-1.0, min(1.0, p + i + d + cte))
        self.prev_error = yaw_err

        cmd = Accel()
        cmd.linear.x = float(TARGET_VELOCITY)
        cmd.angular.z = float(steer)
        self.accel_raw_pub.publish(cmd)

        self.log_counter += 1
        if self.log_counter % 40 == 0:
            print(f"[Driver{self.vehicle_id}] raw V:{TARGET_VELOCITY:.2f}, steer:{steer:.2f}")


# ============================================================
# [Guardian + Mux] subscribes *_accel_raw, publishes final *_accel
# ============================================================
class Problem3GuardianMux(Node):
    def __init__(self):
        super().__init__("problem3_guardian_mux")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---- 차량 목록 ----
        self.VEH_IDS = [1, 2, 3, 4]
        self.TOPICS = {vid: f"/CAV_{vid:02d}" for vid in self.VEH_IDS}

        # ====================================================
        # ✅ 위쪽 충돌구역 중심점 / 반경
        # ====================================================
        self.CENTER = (-2.3342, 2.3073)
        self.RADIUS = 1.50          # IN
        self.EXIT_RADIUS = 0.2     # OUT (히스테리시스)
        self.HYSTERESIS_N = 5

        # ---- 속도 정책 ----
        self.V_NOM = 0.48
        # rank1=0.48, rank2=0.25, rank3=0.10, rank4=0.10(안전)
        self.RANK_SPEEDS = [0.48, 0.34, 0.2, 0.10]

        # limiter/ramp
        self.TICK = 0.05
        self.RAMP_PER_SEC = 0.60
        self.MIN_SPEED = 0.05

        # 상태
        self.pose = {vid: None for vid in self.VEH_IDS}
        self.raw = {vid: Accel() for vid in self.VEH_IDS}

        self.active = {vid: False for vid in self.VEH_IDS}
        self.outside_ticks = {vid: 0 for vid in self.VEH_IDS}

        # 속도 추정(랭킹/TTC용)
        self.last_pose = {vid: None for vid in self.VEH_IDS}
        self.v_est = {vid: self.V_NOM for vid in self.VEH_IDS}

        # lock_order: 토글 방지 핵심 (역순 변경 방지)
        self.lock_order = []  # 예: [3,1,2] 처럼 유지

        # 현재 제한(램프)
        self.cmd_limit = {vid: 99.0 for vid in self.VEH_IDS}
        self.tgt_limit = {vid: None for vid in self.VEH_IDS}

        # sub/pub
        for vid in self.VEH_IDS:
            topic = self.TOPICS[vid]
            self.create_subscription(PoseStamped, topic, self._make_pose_cb(vid), qos)
            self.create_subscription(Accel, f"{topic}_accel_raw", self._make_raw_cb(vid), 10)

        self.pub = {vid: self.create_publisher(Accel, f"{self.TOPICS[vid]}_accel", 10)
                    for vid in self.VEH_IDS}

        self._log_counter = 0
        self.create_timer(self.TICK, self.tick)
        self.get_logger().info(
            "✅ P3 GuardianMux ready: ranking+lock_order(no reverse toggle), ignores {1,2}-only case"
        )

    def _make_pose_cb(self, vid):
        def cb(msg):
            p = (msg.pose.position.x, msg.pose.position.y)
            self.pose[vid] = p
            self._update_speed_est(vid, p)
        return cb

    def _make_raw_cb(self, vid):
        def cb(msg):
            self.raw[vid] = msg
        return cb

    def _update_speed_est(self, vid, curr_p):
        prev = self.last_pose[vid]
        if prev is not None:
            dist = math.hypot(curr_p[0] - prev[0], curr_p[1] - prev[1])
            v_now = dist / self.TICK
            # 약간 smoothing
            self.v_est[vid] = 0.3 * v_now + 0.7 * self.v_est[vid]
        self.last_pose[vid] = curr_p

    def _apply_limit(self, vid, tgt):
        # tgt None => no limit
        if tgt is None:
            self.cmd_limit[vid] = 99.0
            return

        cur = self.cmd_limit[vid]
        if cur > 50:
            cur = self.V_NOM

        step = self.RAMP_PER_SEC * self.TICK
        if tgt > cur:
            cur = min(tgt, cur + step)
        else:
            cur = max(tgt, cur - step)

        cur = max(self.MIN_SPEED, cur)
        self.cmd_limit[vid] = cur

    def _update_active_flags(self):
        for vid in self.VEH_IDS:
            p = self.pose[vid]
            if p is None:
                continue

            d = math.hypot(p[0] - self.CENTER[0], p[1] - self.CENTER[1])

            if d < self.RADIUS:
                self.active[vid] = True
                self.outside_ticks[vid] = 0
            elif d > self.EXIT_RADIUS:
                if self.active[vid]:
                    self.outside_ticks[vid] += 1
                    if self.outside_ticks[vid] >= self.HYSTERESIS_N:
                        self.active[vid] = False
                        self.outside_ticks[vid] = 0

    def _rank_by_ttc(self, in_zone):
        scored = []
        for vid in in_zone:
            p = self.pose[vid]
            if p is None:
                continue
            dist = math.hypot(p[0] - self.CENTER[0], p[1] - self.CENTER[1])
            v = max(0.05, float(self.v_est[vid]))
            ttc = dist / v
            scored.append((ttc, vid))
        scored.sort(key=lambda x: x[0])  # 작은 ttc가 먼저(더 급함)
        return [vid for _, vid in scored] if scored else list(in_zone)

    def _update_lock_order(self, in_zone):
        """
        ✅ 토글(역순) 방지 핵심:
        1) 나간 차량은 lock_order에서 제거
        2) lock_order가 비었을 때만 TTC로 초기 순서 1회 결정
        3) 새로 들어온 차량은 '뒤'에 append (기존 순서 유지)
        """
        in_set = set(in_zone)

        # 1) 나간 차량 제거
        self.lock_order = [vid for vid in self.lock_order if vid in in_set]

        # 2) 비었으면 TTC 초기화(1회)
        if not self.lock_order and in_zone:
            self.lock_order = self._rank_by_ttc(list(in_zone))

        # 3) 새로 들어온 차 뒤에 추가
        for vid in in_zone:
            if vid not in self.lock_order:
                self.lock_order.append(vid)

    def tick(self):
        if all(self.pose[vid] is None for vid in self.VEH_IDS):
            return

        # 1) IN/OUT 갱신
        self._update_active_flags()

        # 2) in_zone 계산
        in_zone = [vid for vid in self.VEH_IDS if self.active[vid] and (self.pose[vid] is not None)]
        in_set = set(in_zone)

        # ✅ 조건: 2대 이상 들어와야 작동
        # ✅ 추가 조건: (1,2)만 들어온 경우는 OFF (둘은 충돌 안 함)
        algo_on = (len(in_zone) >= 2) and not (in_set == {1, 2})

        # 3) 기본 제한 없음
        for vid in self.VEH_IDS:
            self.tgt_limit[vid] = None

        priority_list = []
        if algo_on:
            # lock_order 업데이트(토글 방지)
            self._update_lock_order(in_zone)
            # zone 안에 있는 순서만 추출
            priority_list = [vid for vid in self.lock_order if vid in in_set]

            # rank별 속도 적용 (rank1=0.48, rank2=0.25, rank3=0.10 ...)
            for rank, vid in enumerate(priority_list):
                if rank < len(self.RANK_SPEEDS):
                    lim = self.RANK_SPEEDS[rank]
                    # 1등 속도가 V_NOM이면 제한 None 처리(그대로 달림)
                    self.tgt_limit[vid] = None if lim >= self.V_NOM else float(lim)

        else:
            # 알고리즘 OFF면 lock_order 비우지 말고 유지해도 되지만,
            # 원하면 아래 한 줄로 “구역 비면 리셋” 가능:
            if len(in_zone) == 0:
                self.lock_order = []

        # 4) limiter 적용
        for vid in self.VEH_IDS:
            self._apply_limit(vid, self.tgt_limit[vid])

        # 5) 최종 publish (raw speed vs limit)
        for vid in self.VEH_IDS:
            raw_v = float(self.raw[vid].linear.x)
            lim = float(self.cmd_limit[vid])
            final_v = min(raw_v, lim)

            out = Accel()
            out.linear.x = float(final_v)
            out.angular.z = float(self.raw[vid].angular.z)
            self.pub[vid].publish(out)

        # 6) 로그 (✅ f-string 조건부 포맷 제거 버전)
        self._log_counter += 1
        if self._log_counter % 20 == 0:
            leader = priority_list[0] if (algo_on and priority_list) else None
            order_txt = ",".join(str(x) for x in priority_list) if priority_list else "-"
            parts = []
            for vid in self.VEH_IDS:
                act = "IN" if self.active[vid] else "--"
                raw_v = float(self.raw[vid].linear.x)
                lim = float(self.cmd_limit[vid])
                out_v = min(raw_v, lim)
                parts.append(f"{vid}:{act} raw={raw_v:.2f} lim={lim:.2f} out={out_v:.2f}")

            print(f"[P3] algo={'ON' if algo_on else 'OFF'} leader={leader} order=[{order_txt}] | " + " | ".join(parts))


# ============================================================
# main
# ============================================================
def main():
    rclpy.init(args=None)

    nodes = [
        ZonePriorityDriver(1),
        ZonePriorityDriver(2),
        ZonePriorityDriver(3),
        ZonePriorityDriver(4),
        Problem3GuardianMux(),
    ]

    ex = MultiThreadedExecutor(num_threads=8)
    for n in nodes:
        ex.add_node(n)

    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


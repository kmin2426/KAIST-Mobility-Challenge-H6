#!/usr/bin/env python3
import os
import csv
import math
import sys
import importlib.util

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Accel, PoseStamped


# ============================================================
# [0] 유틸: 같은 py 파일을 "다른 모듈 이름"으로 2번 로드
# ============================================================
def load_driver_module_as(module_name: str, py_path: str, target_vehicle_id: int):
    old_argv = sys.argv[:]
    try:
        # 원본 파일이 sys.argv[1]로 차량 id를 읽는 구조라서 이렇게 우회
        sys.argv = [py_path, str(target_vehicle_id)]
        spec = importlib.util.spec_from_file_location(module_name, py_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        sys.argv = old_argv


# ============================================================
# [1] DZ 로드/거리 계산
# ============================================================
def load_dz_points(csv_file: str):
    pts = []
    if not os.path.exists(csv_file):
        return pts

    with open(csv_file, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0]); y = float(row[1])
                if math.isfinite(x) and math.isfinite(y):
                    pts.append((x, y))
            except:
                pass
    return pts


def min_dist_to_dz(x: float, y: float, pts):
    md = float("inf")
    for zx, zy in pts:
        d = math.hypot(zx - x, zy - y)
        if d < md:
            md = d
    return md


# ============================================================
# [2] Guardian 노드 (DZ1 + DZ2)
#     - DZ2: "교차점까지 더 가까운 차량"이 우선 (고정 우선권 제거)
#     - encounter 고정(역정지 방지): 둘 다 DZ 안에 있을 때만 정지 유지
# ============================================================
class MultiZoneGuardian(Node):
    def __init__(self):
        super().__init__("multi_zone_guardian")

        # ----------------------------
        # 파일명 (폴더에 있는 이름으로!)
        # ----------------------------
        self.DZ1_FILE = "right_1_2.csv"
        self.DZ2_FILE = "upper_1_2.csv"

        # ----------------------------
        # 파라미터
        # ----------------------------
        # DZ1
        self.DZ1_NEAR_TOL = 0.40
        self.DZ1_SAFETY_DIST = 1.20
        self.DZ1_EPS_Y = 0.06

        # DZ2 (위쪽 교차점)
        self.DZ2_NEAR_TOL = 0.80   # ✅ near가 안 잡히면 더 키워(0.7~1.5까지 테스트)
        self.DZ2_SAFETY_DIST = 1.40

        # 디버그
        self.DEBUG_EVERY_N = 10

        # ----------------------------
        # QoS / 토픽
        # ----------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub1 = self.create_subscription(PoseStamped, "/CAV_01", self.cb_pose1, qos)
        self.sub2 = self.create_subscription(PoseStamped, "/CAV_02", self.cb_pose2, qos)

        self.pub1 = self.create_publisher(Accel, "/CAV_01_accel", 10)
        self.pub2 = self.create_publisher(Accel, "/CAV_02_accel", 10)

        self.p1 = None
        self.p2 = None

        self.dz1 = load_dz_points(self.DZ1_FILE)
        self.dz2 = load_dz_points(self.DZ2_FILE)

        self.get_logger().info(f"✅ Loaded DZ1: {len(self.dz1)} pts ({self.DZ1_FILE})")
        self.get_logger().info(f"✅ Loaded DZ2: {len(self.dz2)} pts ({self.DZ2_FILE})")

        # ----------------------------
        # DZ2 교차점(근사) 잡기
        # - 너가 DZ2를 "교차점까지" 자른 파일이면,
        #   일반적으로 끝점이 교차점 근처임.
        # - 여기선 "x가 가장 큰 점"을 교차점으로 사용.
        #   (맵에서 교차점이 x최대가 아니라면 key를 바꿔!)
        # ----------------------------
        if self.dz2:
            self.dz2_intersection = max(self.dz2, key=lambda p: p[0])  # x 최대
            self.get_logger().info(f"✅ DZ2 intersection approx = {self.dz2_intersection}")
        else:
            self.dz2_intersection = None
            self.get_logger().warn("⚠️ DZ2 empty; DZ2 logic disabled")

        # ----------------------------
        # encounter 고정 변수
        #   enc_zone: "DZ1" or "DZ2"
        #   enc_yield: 1 or 2 (정지할 차량)
        # ----------------------------
        self.enc_zone = None
        self.enc_yield = None

        self._dbg = 0
        self.timer = self.create_timer(0.02, self.tick)  # 50Hz

    def cb_pose1(self, msg):
        self.p1 = (msg.pose.position.x, msg.pose.position.y)

    def cb_pose2(self, msg):
        self.p2 = (msg.pose.position.x, msg.pose.position.y)

    def publish_stop(self, cav_id: int):
        # ⚠️ 너 환경: 음수 accel -> 후진 폭주. 0으로만 정지 덮어쓰기.
        msg = Accel()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        if cav_id == 1:
            self.pub1.publish(msg)
        else:
            self.pub2.publish(msg)

    def in_zone(self, zone_name: str, x: float, y: float) -> bool:
        if zone_name == "DZ1" and self.dz1:
            return min_dist_to_dz(x, y, self.dz1) <= self.DZ1_NEAR_TOL
        if zone_name == "DZ2" and self.dz2:
            return min_dist_to_dz(x, y, self.dz2) <= self.DZ2_NEAR_TOL
        return False

    def dist_to_dz2_intersection(self, x: float, y: float) -> float:
        """DZ2 교차점까지의 직선거리(근사).
        더 정확히 하려면 경로 arc-length로 바꿀 수 있음."""
        if self.dz2_intersection is None:
            return 999.0
        ix, iy = self.dz2_intersection
        return math.hypot(ix - x, iy - y)

    def tick(self):
        if self.p1 is None or self.p2 is None:
            return

        x1, y1 = self.p1
        x2, y2 = self.p2
        d12 = math.hypot(x1 - x2, y1 - y2)

        # ====================================================
        # [0] encounter 고정 유지 (역정지 방지)
        # - 락이 걸려 있으면:
        #   "둘 다 해당 DZ 안에 있을 때만" yield 차량 stop 유지
        # - 둘 중 하나라도 DZ를 벗어나면 즉시 unlock
        # ====================================================
        if self.enc_zone is not None and self.enc_yield is not None:
            in1 = self.in_zone(self.enc_zone, x1, y1)
            in2 = self.in_zone(self.enc_zone, x2, y2)

            if in1 and in2:
                self.publish_stop(self.enc_yield)
                return
            else:
                # 한 대라도 DZ 밖이면 encounter 종료
                self.enc_zone = None
                self.enc_yield = None
                # 계속 진행해서 새 encounter 판단 가능

        # ====================================================
        # [1] DZ1 (오른쪽 교차점)
        # ====================================================
        if self.dz1:
            md1_1 = min_dist_to_dz(x1, y1, self.dz1)
            md1_2 = min_dist_to_dz(x2, y2, self.dz1)
            near1 = (md1_1 <= self.DZ1_NEAR_TOL)
            near2 = (md1_2 <= self.DZ1_NEAR_TOL)

            if near1 and near2 and (d12 <= self.DZ1_SAFETY_DIST):
                # 기존 너 로직 스타일 유지
                if y1 < y2 - self.DZ1_EPS_Y:
                    yield_id = 2
                elif y2 < y1 - self.DZ1_EPS_Y:
                    yield_id = 1
                else:
                    yield_id = 2

                self.enc_zone = "DZ1"
                self.enc_yield = yield_id
                self.publish_stop(yield_id)
                return

        # ====================================================
        # [2] DZ2 (위쪽 교차점) - 동적 우선권
        #     더 빨리 교차점에 도착하는 차량이 우선(go)
        # ====================================================
        if self.dz2:
            md2_1 = min_dist_to_dz(x1, y1, self.dz2)
            md2_2 = min_dist_to_dz(x2, y2, self.dz2)
            near1 = (md2_1 <= self.DZ2_NEAR_TOL)
            near2 = (md2_2 <= self.DZ2_NEAR_TOL)

            self._dbg += 1
            if self._dbg % self.DEBUG_EVERY_N == 0:
                self.get_logger().info(
                    f"[DZ2] md1={md2_1:.2f} md2={md2_2:.2f} tol={self.DZ2_NEAR_TOL:.2f} "
                    f"d12={d12:.2f} near1={near1} near2={near2}"
                )

            if near1 and near2 and (d12 <= self.DZ2_SAFETY_DIST):
                # 교차점까지 가까운(=더 빨리 도착 가능) 차량이 go
                t1 = self.dist_to_dz2_intersection(x1, y1)
                t2 = self.dist_to_dz2_intersection(x2, y2)

                # tie-break(거의 동일)일 때는 "2번 go"로 둠 (원하면 1로 바꿔)
                if t1 < t2 - 0.05:
                    # 1번이 더 빨리 교차점 도착 => 1 go, 2 yield
                    yield_id = 2
                elif t2 < t1 - 0.05:
                    # 2번이 더 빨리 도착 => 2 go, 1 yield
                    yield_id = 1
                else:
                    yield_id = 2  # tie-break

                self.enc_zone = "DZ2"
                self.enc_yield = yield_id
                self.publish_stop(yield_id)
                return


# ============================================================
# [3] main: 차량1/차량2 원본 드라이버 2개 + guardian 1개 실행
# ============================================================
def main():
    rclpy.init()

    base_py = os.path.join(os.path.dirname(__file__), "my_driver1_2_collision.py")

    mod1 = load_driver_module_as("driver_mod_1", base_py, 1)
    mod2 = load_driver_module_as("driver_mod_2", base_py, 2)

    driver1 = mod1.PriorityRotaryDriver()
    driver2 = mod2.PriorityRotaryDriver()

    guardian = MultiZoneGuardian()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(driver1)
    executor.add_node(driver2)
    executor.add_node(guardian)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        driver1.destroy_node()
        driver2.destroy_node()
        guardian.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


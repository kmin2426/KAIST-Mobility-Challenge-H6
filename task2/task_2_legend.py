import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Accel, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import math
import time
import numpy as np
import os

class MergeCorridorController(Node):
    def __init__(self):
        super().__init__('Task_2')
        self.get_logger().info("🏎️ Task2 Legend Tuning")

        # --- QoS Setup ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- ROS 2 Communication ---
        self.pose_sub = self.create_subscription(
            PoseStamped, '/CAV_01', self.pose_callback, qos_profile)
        
        self.control_pub = self.create_publisher(Accel, '/CAV_01_accel', 10)
        
        self.hv_data = {} 
        for i in range(19, 37):
            self.create_subscription(
                PoseStamped, f'/HV_{i}', 
                lambda msg, h_id=i: self.hv_callback(msg, h_id), 
                qos_profile
            )

        # --- Path Loading ---
        self.paths = {}
        self.paths[1] = self.load_path_file("lane1.json")   
        self.paths[2] = self.load_path_file("lane2.json")   
        self.paths[3] = self.load_path_file("lane3.json") 

        self.path_arrs = {}
        for i in [1, 2, 3]:
            if self.paths[i]['X']:
                self.path_arrs[i] = np.column_stack((self.paths[i]['X'], self.paths[i]['Y']))
            else:
                self.path_arrs[i] = np.empty((0, 2))

        # --- HV ID Configuration ---
        self.LANE_HV_MAP = {
            1: list(range(19, 23)),  # Slow HVs
            2: list(range(23, 31)),  # Fast HVs
            3: list(range(31, 37))   # Static Obstacles
        }

        # --- Merge Corridor Coordinates ---
        # Defines the critical zone where Ego (Lane 1) must yield to Lane 2.
        self.MERGE_CORRIDOR = np.array([
            [5.0583, 0.6667], [5.0618, 0.6573], [5.0654, 0.6479], [5.0689, 0.6386]
        ])

        # --- State Variables ---
        self.current_lane_id = 2
        self.last_nearest_idx = None
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0
        
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_time = time.time()
        self.speed_error_sum = 0.0 

        self.pose_received_count = 0 
        self.is_ready = False        

        self.last_lane_change_time = 0.0
        self.LANE_CHANGE_COOLDOWN = 0.8 
        self.logic_start_time = None 

        # --- Tuning Parameters ---
        self.CAR_LENGTH = 0.21
        self.LANE_WIDTH = 0.24
        self.MAX_SPEED_STRAIGHT = 2.0  
        self.MAX_SPEED_CORNER = 1.05   
        self.MIN_LOOKAHEAD = 0.48      
        self.MAX_LOOKAHEAD = 0.85      
        self.PID_KP = 1.5 
        self.PID_KI = 0.05
        
        self.CORNER_THRESHOLD = 0.15 
        
        # Smoothing Variables
        self.prev_accel_cmd = 0.0
        self.smoothed_target_v = 0.0

        self.create_timer(0.02, self.control_loop)

    def load_path_file(self, filename):
        try:
            if not os.path.exists(filename): return {'X': [], 'Y': []}
            with open(filename, 'r') as f:
                data = json.load(f)
                xs = data.get('X') or data.get('x', [])
                ys = data.get('Y') or data.get('y', [])
                return {'X': xs, 'Y': ys}
        except: return {'X': [], 'Y': []}

    def set_path(self, lane_id):
        if self.path_arrs[lane_id].shape[0] == 0: return
        self.current_lane_id = lane_id
        self.last_nearest_idx = None
        self.speed_error_sum = 0.0
        self.last_lane_change_time = time.time()

    def hv_callback(self, msg, hv_id):
        curr_x = msg.pose.position.x
        curr_y = msg.pose.position.y
        curr_time = time.time()
        if hv_id in self.hv_data:
            prev = self.hv_data[hv_id]
            dt = curr_time - prev['last_time']
            if dt > 0.0:
                dist = math.hypot(curr_x - prev['x'], curr_y - prev['y'])
                v = dist / dt
                fv = 0.4 * prev.get('v', 0.0) + 0.6 * v if v < 10.0 else prev.get('v', 0.0)
            else: fv = prev.get('v', 0.0)
        else: fv = 0.0
        self.hv_data[hv_id] = {'x': curr_x, 'y': curr_y, 'v': fv, 'last_time': curr_time}

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_yaw = msg.pose.orientation.z
        
        curr_time = time.time()
        dt = curr_time - self.prev_time
        if dt > 0:
            dist = math.hypot(self.current_x - self.prev_x, self.current_y - self.prev_y)
            self.current_speed = dist / dt if dist > 0.005 else 0.0
        
        self.prev_x = self.current_x
        self.prev_y = self.current_y
        self.prev_time = curr_time
        
        self.pose_received_count += 1
        if not self.is_ready and self.pose_received_count > 3:
            self.get_logger().info("✅ Task 2 controller activated")
            self.is_ready = True
            if self.logic_start_time is None:
                self.logic_start_time = time.time()

    def get_nearest_index(self):
        path_arr = self.path_arrs[self.current_lane_id]
        if len(path_arr) == 0: return 0
        if self.last_nearest_idx is None:
            dists = np.hypot(path_arr[:,0] - self.current_x, path_arr[:,1] - self.current_y)
            self.last_nearest_idx = np.argmin(dists)
            return self.last_nearest_idx
        
        start_idx = self.last_nearest_idx
        indices = np.arange(start_idx, start_idx + 100) % len(path_arr)
        check_arr = path_arr[indices]
        dists = np.hypot(check_arr[:,0] - self.current_x, check_arr[:,1] - self.current_y)
        self.last_nearest_idx = indices[np.argmin(dists)]
        return self.last_nearest_idx

    # --- Check if Ego is inside the Merge Corridor ---
    def is_in_merge_corridor(self):
        if self.MERGE_CORRIDOR is None: return False
        
        # Calculate distance to all waypoints in the corridor
        dists = np.hypot(self.MERGE_CORRIDOR[:,0] - self.current_x, 
                         self.MERGE_CORRIDOR[:,1] - self.current_y)
        min_dist = np.min(dists)

        # Trigger logic if within 2.0m of the path points
        return min_dist < 2.0

    def get_lane_info(self, target_lane_id):
        min_f_gap, f_vel = 999.0, 0.0
        min_r_gap, r_vel = 999.0, 0.0
        
        path_arr = self.path_arrs[target_lane_id]
        if len(path_arr) == 0: return (999.0, 0.0, 999.0, 0.0)

        dists = np.hypot(path_arr[:,0] - self.current_x, path_arr[:,1] - self.current_y)
        my_idx = np.argmin(dists)
        next_idx = (my_idx + 5) % len(path_arr)
        vec_x = path_arr[next_idx, 0] - path_arr[my_idx, 0]
        vec_y = path_arr[next_idx, 1] - path_arr[my_idx, 1]

        check_ids = list(self.LANE_HV_MAP.get(target_lane_id, []))

        # --- Merge Zone Logic ---
        # If in Lane 1 AND inside the merge corridor, check Lane 2 HVs as well
        in_merge_zone = (target_lane_id == 1) and self.is_in_merge_corridor()
        
        if in_merge_zone:
            check_ids.extend(self.LANE_HV_MAP[2])

        for hv_id in check_ids:
            if hv_id not in self.hv_data: continue
            data = self.hv_data[hv_id]

            dx = data['x'] - self.current_x
            dy = data['y'] - self.current_y
            raw_dist = math.hypot(dx, dy)
            if raw_dist > 4.0: continue

            dot = dx * vec_x + dy * vec_y
            gap = raw_dist - self.CAR_LENGTH

            if dot > 0: # Front Vehicle
                if gap < min_f_gap: 
                    min_f_gap = gap
                    f_vel = data['v']
            else: # Rear Vehicle
                # --- Side Protection ---
                # In merge zone, treat close side/rear Lane 2 cars as frontal obstacles (force yield)
                is_side = in_merge_zone and (hv_id in self.LANE_HV_MAP[2]) and (gap < 0.5)
                if is_side:
                    if gap < min_f_gap:
                        min_f_gap = 0.1 # Fake gap to force braking
                        f_vel = data['v']
                    continue

                # --- Speed Dominance ---
                # Ignore slower rear vehicles (prevents panic braking)
                hv_speed = data['v']
                am_i_faster = self.current_speed > (hv_speed + 0.1)
                
                # Boost Immunity (First 3 seconds)
                is_launching = (self.logic_start_time is not None) and \
                               (time.time() - self.logic_start_time < 3.0)
                
                if is_launching: continue 
                if am_i_faster and gap > 0.5: continue 

                if gap < min_r_gap: 
                    min_r_gap = gap
                    r_vel = hv_speed

        return min_f_gap, f_vel, min_r_gap, r_vel

    def calculate_lane_cost(self, lane_id):
        cost = 0.0
        # Lane 3 is a last resort (Static Obstacles) -> High Cost
        if lane_id == 3: cost += 500.0 

        f_gap, f_vel, r_gap, r_vel = self.get_lane_info(lane_id)
        
        # Safety Cost (TTC)
        if r_gap < 1.2: 
            app_spd = r_vel - self.current_speed
            if app_spd > 0: 
                ttc = r_gap / app_spd
                if ttc < 0.5: cost += 100000.0 
                else: cost += (1.0/ttc) * 2000.0 
            elif r_gap < 0.3: cost += 5000.0

        # Efficiency Cost
        if f_gap < 5.0: 
            if f_gap < 0.3: cost += 5000.0 
            target_diff = self.MAX_SPEED_STRAIGHT - f_vel
            if target_diff > 0: cost += target_diff * 100.0
            
            # Avoid static cars in Lane 3
            if lane_id == 3 and f_vel < 0.1:
                if f_gap < 2.0: cost += (2.0 - f_gap) * 1000.0 
            else:
                cost += (1.0 / max(f_gap, 0.1)) * 40.0

        if lane_id != self.current_lane_id: cost += 10.0
        if lane_id == 2: cost -= 5.0 

        return cost

    def check_future_curvature(self):
        path_arr = self.path_arrs[self.current_lane_id]
        if self.last_nearest_idx is None: return 0.0
        future_lookahead = 40 
        future_idx = (self.last_nearest_idx + future_lookahead) % len(path_arr)
        ld_scan = 0.5
        scan_idx = future_idx
        for _ in range(10): 
            scan_idx = (scan_idx + 1) % len(path_arr)
            dx = path_arr[scan_idx, 0] - path_arr[future_idx, 0]
            dy = path_arr[scan_idx, 1] - path_arr[future_idx, 1]
            if math.hypot(dx, dy) > ld_scan: break
        vec1_x = path_arr[(future_idx+1)%len(path_arr), 0] - path_arr[future_idx, 0]
        vec1_y = path_arr[(future_idx+1)%len(path_arr), 1] - path_arr[future_idx, 1]
        future_yaw = math.atan2(vec1_y, vec1_x)
        tx = path_arr[scan_idx, 0]; ty = path_arr[scan_idx, 1]
        dx = tx - path_arr[future_idx, 0]; dy = ty - path_arr[future_idx, 1]
        local_y = math.sin(-future_yaw) * dx + math.cos(-future_yaw) * dy
        future_curvature = 2.0 * local_y / (ld_scan ** 2)
        return abs(future_curvature)

    # --- Explicit Yield Check for Merge Zone ---
    def check_merge_yield(self):
        # Only active if in Lane 1 AND inside the Merge Corridor
        if not self.is_in_merge_corridor(): return False
        if self.current_lane_id == 2: return False

        # Scan for nearby Fast HVs (Lane 2)
        for hv_id in self.LANE_HV_MAP[2]:
            if hv_id not in self.hv_data: continue
            data = self.hv_data[hv_id]
            
            dx = data['x'] - self.current_x
            dy = data['y'] - self.current_y
            dist = math.hypot(dx, dy)
            
            # If any Lane 2 vehicle is within 0.5m (Side/Rear/Front), YIELD.
            if dist < 0.5:
                return True
        return False

    def decision_logic(self, current_curvature):
        # Lock LC on curves
        if abs(current_curvature) > self.CORNER_THRESHOLD: return
        if self.check_future_curvature() > self.CORNER_THRESHOLD: return 

        costs = {}
        search_lanes = [self.current_lane_id]
        if self.current_lane_id == 1: search_lanes.append(2)
        elif self.current_lane_id == 2: search_lanes.extend([1, 3])
        elif self.current_lane_id == 3: search_lanes.append(2)

        for lane in search_lanes:
            costs[lane] = self.calculate_lane_cost(lane)
        
        best_lane = min(costs, key=costs.get)
        current_cost = costs.get(self.current_lane_id, 99999.0)
        
        time_since = time.time() - self.last_lane_change_time
        is_emergency = current_cost > 50000.0 
        
        if best_lane != self.current_lane_id:
            if is_emergency or (time_since > self.LANE_CHANGE_COOLDOWN):
                margin = 40.0 
                if is_emergency: margin = 0.0
                if current_cost - costs[best_lane] > margin:
                    self.get_logger().info(f"💡 Lane Change: {self.current_lane_id} -> {best_lane}")
                    self.set_path(best_lane)

    def control_loop(self):
        if not self.is_ready:
            stop_msg = Accel(); stop_msg.linear.x = 0.0; stop_msg.angular.z = 0.0
            self.control_pub.publish(stop_msg)
            return

        nearest_idx = self.get_nearest_index()
        path_arr = self.path_arrs[self.current_lane_id]
        
        # Adaptive Lookahead
        current_ld = np.clip(0.48 + 0.15 * self.current_speed, self.MIN_LOOKAHEAD, self.MAX_LOOKAHEAD)
        idx = nearest_idx
        for _ in range(50):
            idx = (idx + 1) % len(path_arr)
            dx = path_arr[idx, 0] - self.current_x
            dy = path_arr[idx, 1] - self.current_y
            if math.hypot(dx, dy) > current_ld: break
        
        tx, ty = path_arr[idx]
        dx = tx - self.current_x; dy = ty - self.current_y
        local_x = math.cos(-self.current_yaw) * dx - math.sin(-self.current_yaw) * dy
        local_y = math.sin(-self.current_yaw) * dx + math.cos(-self.current_yaw) * dy
        curvature = 2.0 * local_y / (current_ld ** 2)
        
        self.decision_logic(curvature)
        
        target_v = self.MAX_SPEED_STRAIGHT
        is_cornering = False
        if abs(curvature) > 0.40:
            is_cornering = True
            brake_factor = min(1.0, (abs(curvature) - 0.40) * 2.5) 
            target_v = self.MAX_SPEED_STRAIGHT * (1.0 - brake_factor * 0.55) 
            target_v = max(target_v, self.MAX_SPEED_CORNER)

        f_gap, f_vel, r_gap, r_vel = self.get_lane_info(self.current_lane_id)
        
        if r_gap < 1.0 and r_vel > self.current_speed: target_v = self.MAX_SPEED_STRAIGHT + 0.3
        
        if f_gap < 0.6: 
            safe_ratio = max(0.0, (f_gap - 0.15) / (0.6 - 0.15))
            target_v = min(target_v, f_vel * 0.9 + target_v * 0.1 * safe_ratio)
        
        # [Override] Merge Yield Logic
        if self.check_merge_yield():
            target_v = 0.0 

        # --- Speed Smoothing ---
        self.smoothed_target_v = 0.8 * self.smoothed_target_v + 0.2 * target_v
        
        error = self.smoothed_target_v - self.current_speed
        accel_cmd = self.PID_KP * error + self.PID_KI * self.speed_error_sum
        
        # --- Soft Boost Launch ---
        if self.current_speed < 0.5 and self.smoothed_target_v > 0.5 and not is_cornering:
            accel_cmd = max(accel_cmd, 1.5)
        else:
            if self.current_speed < 0.1 and self.smoothed_target_v > 0.1 and accel_cmd < 0.6:
                accel_cmd = 0.8 

        final_accel = float(accel_cmd)
        
        # --- Acceleration LPF ---
        alpha = 0.3 
        final_accel = alpha * final_accel + (1.0 - alpha) * self.prev_accel_cmd
        self.prev_accel_cmd = final_accel

        # --- Safety Clamp ---
        if self.current_speed < 0.1 and final_accel < 0.0: final_accel = 0.0
        final_accel = max(final_accel, -1.0)

        # --- Lateral Control ---
        nx = path_arr[nearest_idx, 0] - self.current_x
        ny = path_arr[nearest_idx, 1] - self.current_y
        cte_y = math.sin(-self.current_yaw) * nx + math.cos(-self.current_yaw) * ny
        
        k_cte = 1.1
        if self.current_lane_id == 2 and cte_y < -0.15: k_cte = 2.5 

        final_steer = (curvature * 1.0) + (cte_y * k_cte)

        cmd = Accel()
        cmd.linear.x = final_accel
        cmd.angular.z = float(final_steer)
        self.control_pub.publish(cmd)

def main():
    rclpy.init()
    node = MergeCorridorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
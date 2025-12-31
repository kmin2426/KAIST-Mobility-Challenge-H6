import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Accel, PoseStamped
from rclpy.qos import qos_profile_sensor_data
import json
import math
import time

##### Hyperparameters #####
LOOKAHEAD_DIST = 0.3   
SPEED = 0.5
STEER_GAIN = 1.0       
STEER_FILTER_ALPHA = 0.7 
CTE_GAIN = 1.5 

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.pose_sub = self.create_subscription(
            PoseStamped, '/Ego_pose', self.pose_callback, qos_profile_sensor_data)

        self.control_pub = self.create_publisher(Accel, '/Accel', 10)
        self.path_x, self.path_y = [], []
        self.load_path()

        self.last_nearest_idx = 0 
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.is_pose_received = False
        self.prev_angular_vel = 0.0
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Tuned Precision Mode ON (L={LOOKAHEAD_DIST}, G={STEER_GAIN}, CTE={CTE_GAIN})")

    def load_path(self):
        try:
            with open("path_1_1.json", 'r') as f:
                data = json.load(f)
                self.path_x = data['X']
                self.path_y = data['Y']
        except Exception as e: pass

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        curr_time = time.time()
        dt = curr_time - self.prev_time
        if dt > 0:
            dist = math.hypot(self.current_x - self.prev_x, self.current_y - self.prev_y)
            self.current_speed = dist / dt
        
        self.prev_x = self.current_x
        self.prev_y = self.current_y
        self.prev_time = curr_time
        self.is_pose_received = True

    def check_reset_condition(self):
        if not self.path_x: return
        start_x = self.path_x[0]
        start_y = self.path_y[0]
        dist_to_start = math.hypot(self.current_x - start_x, self.current_y - start_y)

        if self.last_nearest_idx > 100 and dist_to_start < 0.5:
            self.get_logger().warn(">>> R key detected! Resetting... <<<")
            self.last_nearest_idx = 0
            self.prev_angular_vel = 0.0
            self.current_speed = 0.0

    def get_nearest_index(self):
        self.check_reset_condition()
        search_range = 200 
        start_idx = self.last_nearest_idx
        end_idx = min(start_idx + search_range, len(self.path_x))
        dx = [self.current_x - icx for icx in self.path_x[start_idx:end_idx]]
        dy = [self.current_y - icy for icy in self.path_y[start_idx:end_idx]]
        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
        if not d: return self.last_nearest_idx
        min_d = min(d)
        nearest_idx = start_idx + d.index(min_d)
        self.last_nearest_idx = nearest_idx
        return nearest_idx

    def control_loop(self):
        if not self.is_pose_received or not self.path_x: return

        nearest_idx = self.get_nearest_index()

        ##### Pure Pursuit #####
        # Adaptive Lookahead Distance
        current_lookahead = max(LOOKAHEAD_DIST, self.current_speed * 0.5)
        current_lookahead = min(current_lookahead, 0.7)

        target_idx = nearest_idx
        while target_idx < len(self.path_x) - 1:
            dx = self.path_x[target_idx] - self.current_x
            dy = self.path_y[target_idx] - self.current_y
            if math.hypot(dx, dy) > current_lookahead: 
                break
            target_idx += 1

        # dx, dy in vehicle coordinate
        tx = self.path_x[target_idx]
        ty = self.path_y[target_idx]
        dx = tx - self.current_x
        dy = ty - self.current_y

        # Transform to Body Frame
        local_y = math.sin(-self.current_yaw) * dx + math.cos(-self.current_yaw) * dy

        # Curvature
        curvature = 2.0 * local_y / (current_lookahead ** 2)
        
        ##### CTE Calculation #####
        # dx, dy to nearest path point
        near_dx = self.path_x[nearest_idx] - self.current_x
        near_dy = self.path_y[nearest_idx] - self.current_y
        
        # Transform to Body Frame
        cte_y = (math.sin(-self.current_yaw) * near_dx + math.cos(-self.current_yaw) * near_dy) 

        ##### Steering Control #####
        pp_steer = curvature * STEER_GAIN
        cte_steer = cte_y * CTE_GAIN
        final_steer_cmd = pp_steer + cte_steer
        
        # 각속도로 변환 (v * k)
        raw_angular_vel = self.current_speed * final_steer_cmd
        
        # 최종 각속도에 저역 필터 적용
        filtered_angular_vel = (STEER_FILTER_ALPHA * raw_angular_vel) + ((1.0 - STEER_FILTER_ALPHA) * self.prev_angular_vel)
        self.prev_angular_vel = filtered_angular_vel

        cmd_msg = Accel()
        cmd_msg.linear.x = float(SPEED)
        cmd_msg.angular.z = float(filtered_angular_vel)
        self.control_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
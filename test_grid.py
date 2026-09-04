import argparse
import math
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool, Trigger

PEAK_FACTOR = 35.0 / 16.0  # velocity_factor at peak (tau=0.5)

def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def clamp(v, low, high):
    return max(low, min(high, v))


def world_to_body(vx_world, vy_world, yaw):
    # rotate world velocity into robot body frame (omni only)
    c, s = math.cos(yaw), math.sin(yaw)
    vx_body = vx_world * c + vy_world * s
    vy_body = -vx_world * s + vy_world * c
    return vx_body, vy_body


def position_factor(tau):
    tau = clamp(tau, 0.0, 1.0)
    return 35 * tau**4 - 84 * tau**5 + 70 * tau**6 - 20 * tau**7


def velocity_factor(tau):
    tau = clamp(tau, 0.0, 1.0)
    return 140 * tau**3 - 420 * tau**4 + 420 * tau**5 - 140 * tau**6


def duration_for_peak(distance, max_speed):
    distance = abs(distance)
    if distance <= 0 or max_speed <= 0:
        return 0.0
    return PEAK_FACTOR * distance / max_speed


class GridMotionNode(Node):
    def __init__(self, args):
        super().__init__('grid_motion_node')
        self.a = args
        self.x = self.y = self.yaw = 0.0
        self.have_odom = False
        self.home = None
        self.intended_yaw = None 

        # execution state machine
        self.plan = []           # queue of command dictionaries
        self.current_cmd = None  # command being executed
        self.cmd_start_time = 0.0
        self.cmd_start_pose = (0.0, 0.0, 0.0)

        # rate limiting and filtering
        self.last_vx = self.last_vy = self.last_wz = 0.0
        self.last_send_time = None
        self.corr_state = {}
        self.corr_filtered = {}
        self.odom_seq = 0
        self.is_finished = False

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, qos)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos)
        self.motor_client = self.create_client(SetBool, args.motor_power_service)
        self.reset_client = self.create_client(Trigger, args.reset_odom_service)
        self.timer_period = 1.0 / self.a.rate
        self.control_timer = None 

    def get_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True
        self.odom_seq += 1

    def pose(self):
        return self.x, self.y, self.yaw

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        now = self.get_time_sec()
        dt = (now - self.last_send_time) if self.last_send_time else self.timer_period
        self.last_send_time = now        
        max_dv = self.a.max_accel * dt
        max_dw = self.a.max_ang_accel * dt
        vx = clamp(vx, self.last_vx - max_dv, self.last_vx + max_dv)
        vy = clamp(vy, self.last_vy - max_dv, self.last_vy + max_dv)
        wz = clamp(wz, self.last_wz - max_dw, self.last_wz + max_dw)
        self.last_vx, self.last_vy, self.last_wz = vx, vy, wz

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.pub.publish(msg)

    def reset_correction(self):
        self.corr_state.clear()
        self.corr_filtered.clear()

    def tracking_correction(self, axis, error, direction_sign, gain, damping_gain, max_speed, tol, homing=False):
        now = self.get_time_sec()
        seq = self.odom_seq
        prev = self.corr_state.get(axis)
        d_error = 0.0
        
        if prev is None:
            self.corr_state[axis] = (error, now, seq)
        else:
            prev_error, prev_time, prev_seq = prev
            if seq != prev_seq:
                dt = now - prev_time
                if dt > 1e-6:
                    d_error = (error - prev_error) / dt
                    d_error = clamp(d_error, -self.a.corr_max_d_error, self.a.corr_max_d_error)
                self.corr_state[axis] = (error, now, seq)

        if abs(error) <= tol:
            raw_target = 0.0
        else:
            raw_target = gain * error + damping_gain * d_error
            if homing:
                lo, hi = -0.3 * max_speed, 0.3 * max_speed
            elif direction_sign > 0:
                lo, hi = -0.05 * max_speed, 0.3 * max_speed
            elif direction_sign < 0:
                lo, hi = -0.3 * max_speed, 0.05 * max_speed
            else:
                lo, hi = -0.05 * max_speed, 0.05 * max_speed
            raw_target = clamp(raw_target, lo, hi)

        prev_filt = self.corr_filtered.get(axis)
        if prev_filt is None or self.a.corr_filter_tau <= 0:
            filtered = raw_target
        else:
            prev_val, prev_t = prev_filt
            dt_f = now - prev_t
            alpha = clamp(dt_f / (self.a.corr_filter_tau + dt_f), 0.0, 1.0)
            filtered = prev_val + (raw_target - prev_val) * alpha
            
        self.corr_filtered[axis] = (filtered, now)
        return filtered

    # planner

    def plan_wait(self, duration):
        self.plan.append({'type': 'wait', 'duration': duration})

    def plan_turn(self, target_yaw):
        self.plan.append({'type': 'turn', 'target_yaw': target_yaw})

    def plan_turn_by(self, angle):
        if self.intended_yaw is None:
            _, _, self.intended_yaw = self.pose()
        self.intended_yaw = normalize(self.intended_yaw + angle)
        self.plan_turn(self.intended_yaw)

    def plan_drive(self, distance):
        self.plan.append({'type': 'drive', 'distance': distance})

    def plan_move_to(self, target_x, target_y):
        self.plan.append({'type': 'move_to', 'target_x': target_x, 'target_y': target_y})

    def generate_grid_plan(self):
        self.plan.clear()
        n = self.a.grid_size
        cell = self.a.cell_size
        self.get_logger().info(f'Generating {n}x{n} grid plan ({self.a.platform})...')

        self.plan_wait(0.3)

        if self.a.platform == 'omni':
            hx, hy, _ = self.home
            waypoints = []
            for row in range(n):
                wy = hy + row * cell
                cols = range(n) if row % 2 == 0 else reversed(range(n))
                for col in cols:
                    waypoints.append((hx + col * cell, wy))
            for wx, wy in waypoints[1:]:
                self.plan_move_to(wx, wy)
        else:
            for row in range(n):
                for _ in range(n - 1):
                    self.plan_drive(cell)
                if row < n - 1:
                    turn = -math.pi / 2 if row % 2 == 0 else math.pi / 2
                    self.plan_turn_by(turn)
                    self.plan_drive(cell)
                    self.plan_turn_by(turn)

        if not self.a.no_return_home:
            hx, hy, hyaw = self.home
            if self.a.platform == 'omni':
                self.plan_move_to(hx, hy)
                self.plan_turn(hyaw)
            else:
                self.plan.append({'type': 'return_home_diff'})

        self.plan_wait(0.8)

    # controller

    def start_control_loop(self):
        self.generate_grid_plan()
        self.control_timer = self.create_timer(self.timer_period, self.control_tick)

    def control_tick(self):
        if self.is_finished:
            return

        # if no current command pull next one from plan
        if self.current_cmd is None:
            if not self.plan:
                self.get_logger().info('Plan complete.')
                self.send(0.0, 0.0, 0.0) # stop
                self.is_finished = True
                return
            
            self.current_cmd = self.plan.pop(0)
            self.cmd_start_time = self.get_time_sec()
            self.cmd_start_pose = self.pose()
            self.reset_correction()
            self.get_logger().info(f"Executing: {self.current_cmd['type']}")

        # execute current command
        cmd = self.current_cmd
        ctype = cmd['type']
        now = self.get_time_sec()
        elapsed = now - self.cmd_start_time
        x, y, yaw = self.pose()
        start_x, start_y, start_yaw = self.cmd_start_pose
        a = self.a

        if ctype == 'wait':
            self.send(0.0, 0.0, 0.0)
            if elapsed >= cmd['duration']:
                self.current_cmd = None

        elif ctype == 'turn':
            angle = normalize(cmd['target_yaw'] - start_yaw)
            T = duration_for_peak(angle, a.turn_speed)
            if T <= 0:
                self.current_cmd = None
                return

            homing = elapsed >= T
            tau = min(elapsed / T, 1.0)
            w_ff = 0.0 if homing else (angle / T) * velocity_factor(tau)
            
            turned = normalize(yaw - start_yaw)
            expected = angle * position_factor(tau)
            error = expected - turned
            direction_sign = 1 if angle > 0 else -1
            
            w_correction = self.tracking_correction('turn', error, direction_sign, a.corr_gain, a.corr_damping_gain,
                                                     a.turn_speed, a.angle_tol, homing=homing)
            self.send(wz=w_ff + w_correction)

            if homing and abs(error) <= a.angle_tol:
                self.last_wz = 0.0
                self.current_cmd = None
            elif elapsed >= T + a.homing_timeout:
                self.get_logger().warn(f'turn timed out, residual error {math.degrees(error):.2f} deg')
                self.last_wz = 0.0
                self.current_cmd = None

        elif ctype == 'drive':
            dist = cmd['distance']
            T = duration_for_peak(dist, a.max_speed)
            if T <= 0:
                self.current_cmd = None
                return

            homing = elapsed >= T
            tau = min(elapsed / T, 1.0)
            v_ff = 0.0 if homing else (dist / T) * velocity_factor(tau)

            dx = x - start_x
            dy = y - start_y
            travelled = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
            lateral = -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)

            expected = dist * position_factor(tau)
            error = expected - travelled

            v_correction = self.tracking_correction('drive', error, 1, a.corr_gain, a.corr_damping_gain
                                                    , a.max_speed, a.position_tol, homing=homing)
            lateral_correction = self.tracking_correction('drive_lat', -lateral, 0, a.corr_gain, a.corr_damping_gain,
                                                           a.turn_speed, a.position_tol, homing=homing)
            heading_fix = clamp(1.8 * normalize(start_yaw - yaw) + lateral_correction, -0.5 * a.turn_speed, 0.5 * a.turn_speed)
            self.send(vx=v_ff + v_correction, wz=heading_fix)

            if homing and abs(error) <= a.position_tol:
                self.last_vx = 0.0
                self.current_cmd = None
            elif elapsed >= T + a.homing_timeout:
                self.get_logger().warn(f'drive timed out, residual error {error:.3f}m')
                self.last_vx = 0.0
                self.current_cmd = None

        elif ctype == 'move_to':
            dx = cmd['target_x'] - start_x
            dy = cmd['target_y'] - start_y
            T = max(duration_for_peak(dx, a.max_speed), duration_for_peak(dy, a.max_speed))
            if T <= 0:
                self.current_cmd = None
                return

            homing = elapsed >= T
            tau = min(elapsed / T, 1.0)
            vf = 0.0 if homing else velocity_factor(tau)
            pf = position_factor(tau)
            
            expected_x, expected_y = start_x + dx * pf, start_y + dy * pf
            err_x, err_y = expected_x - x, expected_y - y
            
            vx_corr = self.tracking_correction('move_x', err_x, 1 if dx > 0 else (-1 if dx < 0 else 0), a.corr_gain,
                                                a.corr_damping_gain, a.max_speed, a.position_tol, homing=homing)
            vy_corr = self.tracking_correction('move_y', err_y, 1 if dy > 0 else (-1 if dy < 0 else 0), a.corr_gain,
                                                a.corr_damping_gain, a.max_speed, a.position_tol, homing=homing)
            
            vx_world = (dx / T) * vf + vx_corr
            vy_world = (dy / T) * vf + vy_corr
            vx_body, vy_body = world_to_body(vx_world, vy_world, yaw)
            heading_fix = clamp(1.8 * normalize(start_yaw - yaw), -0.5 * a.turn_speed, 0.5 * a.turn_speed)
            
            self.send(vx=vx_body, vy=vy_body, wz=heading_fix)

            if homing and math.hypot(err_x, err_y) <= a.position_tol:
                self.last_vx = self.last_vy = 0.0
                self.current_cmd = None
            elif elapsed >= T + a.homing_timeout:
                self.get_logger().warn(f'move_to timed out, residual error {math.hypot(err_x, err_y):.3f}m')
                self.last_vx = self.last_vy = 0.0
                self.current_cmd = None
                
        elif ctype == 'return_home_diff':
            hx, hy, hyaw = self.home
            dist = math.hypot(hx - x, hy - y)
            retries = cmd.get('retries', 0)
            if dist > a.return_position_tol and retries < a.return_home_max_retries:
                target_angle = math.atan2(hy - y, hx - x)
                self.plan.insert(0, {'type': 'return_home_diff', 'retries': retries + 1})
                self.plan.insert(0, {'type': 'drive', 'distance': dist})
                self.plan.insert(0, {'type': 'turn', 'target_yaw': target_angle})
            else:
                if dist > a.return_position_tol:
                    self.get_logger().warn(f'return_home_diff: giving up after {retries} attempts, residual distance {dist:.3f}m')
                self.plan.insert(0, {'type': 'turn', 'target_yaw': hyaw})
            self.current_cmd = None


# setup scripts

def initialize_robot(node):
    # wait odometry
    node.get_logger().info('waiting for /odom')
    while rclpy.ok() and not node.have_odom:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # motor power service
    if node.motor_client.wait_for_service(timeout_sec=3.0):
        future = node.motor_client.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(node, future)
    else:
        node.get_logger().warn('motor power service not found')

    # reset odom service
    if node.reset_client.wait_for_service(timeout_sec=2.0):
        future = node.reset_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future)
    
    node.home = node.pose()
    node.intended_yaw = node.home[2]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--platform', choices=['diff', 'omni'], default='diff')
    p.add_argument('--grid-size', type=int, default=4)
    p.add_argument('--cell-size', type=float, default=0.25)
    p.add_argument('--max-speed', type=float, default=0.15)
    p.add_argument('--turn-speed', type=float, default=0.25)
    p.add_argument('--corr-gain', type=float, default=1.5)
    p.add_argument('--corr-damping-gain', type=float, default=0.3)
    p.add_argument('--corr-filter-tau', type=float, default=0.15)
    p.add_argument('--position-tol', type=float, default=0.01)
    p.add_argument('--return-position-tol', type=float, default=0.025)
    p.add_argument('--angle-tol', type=float, default=0.025)
    p.add_argument('--rate', type=float, default=25.0)
    p.add_argument('--homing-timeout', type=float, default=2.0)
    p.add_argument('--return-home-max-retries', type=int, default=8)
    p.add_argument('--max-accel', type=float, default=0.3)
    p.add_argument('--max-ang-accel', type=float, default=0.5)
    p.add_argument('--no-return-home', action='store_true')
    p.add_argument('--cmd-vel-topic', default='/cmd_vel')
    p.add_argument('--motor-power-service', default='/motor_power')
    p.add_argument('--reset-odom-service', default='/reset_odometry')
    p.add_argument('--corr-max-d-error', type=float, default=2.0)
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = GridMotionNode(args)
    try:
        initialize_robot(node)
        node.start_control_loop()        
        while rclpy.ok() and not node.is_finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().warn('stopping robot')
    except Exception as exc:
        node.get_logger().error(f'error: {exc}')
        raise
    finally:
        # emergency stop
        node.last_vx = node.last_vy = node.last_wz = 0.0
        for _ in range(10):
            node.send(0.0, 0.0, 0.0)
            try:
                rclpy.spin_once(node, timeout_sec=0.02)
            except Exception:
                break
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
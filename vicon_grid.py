import argparse
import math
import time
import rclpy
from collections import deque
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool

PEAK_FACTOR = 35.0 / 16.0  # velocity_factor at peak (tau=0.5)
HEADING_GAIN = 1.8         # yaw P-gain for heading hold
LATERAL_GAIN = 2.0         # lateral offset to yaw error conversion
LATERAL_CLAMP = 0.35       # max lateral error [m]
HEADING_FIX_LIMIT = 0.5    # heading_fix clamp as fraction of turn_speed


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
        super().__init__('grid_motion_vicon_node')
        self.a = args
        self.x = self.y = self.yaw = 0.0
        self.have_pose = False
        self.home = None
        self.intended_yaw = None

        # execution state machine
        self.plan = deque()      # queue of command dictionaries
        self.current_cmd = None  # command being executed
        self.cmd_start_time = 0.0
        self.cmd_start_pose = (0.0, 0.0, 0.0)

        # rate limiting and filtering
        self.last_vx = self.last_vy = self.last_wz = 0.0
        self.last_send_time = None
        self.corr_state = {}
        self.corr_filtered = {}
        self.pose_seq = 0
        self.vicon_prev = None   # (x, y, yaw, stamp) of previous vicon sample
        self.vicon_v = 0.0
        self.vicon_wz = 0.0
        self.last_vicon_time = None
        self.pose_stamp = None
        self.is_finished = False

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, qos)
        self.create_subscription(PoseStamped, args.vicon_topic, self.vicon_cb, qos)
        self.motor_client = self.create_client(SetBool, args.motor_power_service)
        self.timer_period = 1.0 / self.a.rate
        self.control_timer = None

    def get_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def vicon_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # velocity from filtered finite differences of consecutive vicon samples
        prev = self.vicon_prev
        if prev is not None and 0.0 < stamp - prev[3] < 0.2:
            dt = stamp - prev[3]
            v_raw = math.hypot(x - prev[0], y - prev[1]) / dt
            w_raw = normalize(yaw - prev[2]) / dt
            alpha = dt / (self.a.vel_filter_tau + dt)
            self.vicon_v = alpha * v_raw + (1.0 - alpha) * self.vicon_v
            self.vicon_wz = alpha * w_raw + (1.0 - alpha) * self.vicon_wz
        else:
            self.vicon_v = 0.0
            self.vicon_wz = 0.0
        self.vicon_prev = (x, y, yaw, stamp)
        self.x = x
        self.y = y
        self.yaw = yaw
        self.have_pose = True
        self.last_vicon_time = self.get_time_sec()
        self.pose_stamp = stamp
        self.pose_seq += 1

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

    def tracking_correction(self, axis, error, gain, damping_gain, max_speed, tol):
        now = self.get_time_sec()
        seq = self.pose_seq
        # D-term time base: vicon message stamps (fallback: node time)
        stamp = self.pose_stamp if self.pose_stamp is not None else now
        prev = self.corr_state.get(axis)
        d_error = 0.0

        if prev is None:
            self.corr_state[axis] = (error, stamp, now, seq)
        else:
            prev_error, prev_stamp, prev_now, prev_seq = prev
            if seq != prev_seq:
                dt = stamp - prev_stamp if stamp > prev_stamp else now - prev_now
                if dt > 1e-6:
                    d_error = (error - prev_error) / dt
                    d_error = clamp(d_error, -self.a.corr_max_d_error, self.a.corr_max_d_error)
                self.corr_state[axis] = (error, stamp, now, seq)

        if abs(error) <= tol:
            raw_target = 0.0
        else:
            raw_target = gain * error + damping_gain * d_error
            raw_target = clamp(raw_target, -max_speed, max_speed)

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
        self.plan.append({'type': 'turn', 'target_yaw': self.intended_yaw,
                        'spin': 1.0 if angle >= 0 else -1.0})

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

        now = self.get_time_sec()

        # vicon staleness watchdog
        if self.last_vicon_time is not None and now - self.last_vicon_time > self.a.vicon_timeout:
            self.get_logger().error(f'vicon pose stale for {now - self.last_vicon_time:.1f}s, aborting')
            self.send(0.0, 0.0, 0.0)
            self.is_finished = True
            return

        # if no current command pull next one from plan
        if self.current_cmd is None:
            if not self.plan:
                fx, fy, fyaw = self.pose()
                self.get_logger().info(
                    f'FINAL: dx={fx - self.home[0]:+.3f}m dy={fy - self.home[1]:+.3f}m '
                    f'dyaw={math.degrees(normalize(fyaw - self.home[2])):+.2f} deg')
                self.get_logger().info('Plan complete.')
                self.send(0.0, 0.0, 0.0)
                self.is_finished = True
                return

            self.current_cmd = self.plan.popleft()
            self.cmd_start_time = now
            self.cmd_start_pose = self.pose()
            self.reset_correction()
            self.get_logger().info(f"Executing: {self.current_cmd['type']}")

        # execute current command
        cmd = self.current_cmd
        ctype = cmd['type']
        elapsed = now - self.cmd_start_time
        x, y, yaw = self.pose()
        start_x, start_y, start_yaw = self.cmd_start_pose
        a = self.a

        if ctype == 'wait':
            self.send(0.0, 0.0, 0.0)
            if elapsed >= cmd['duration']:
                self.current_cmd = None

        elif ctype == 'turn':
            raw_angle = cmd['target_yaw'] - start_yaw
            spin = cmd.get('spin', 0.0)
            if spin > 0 and raw_angle < 0:
                raw_angle += 2 * math.pi
            elif spin < 0 and raw_angle > 0:
                raw_angle -= 2 * math.pi
            angle = raw_angle if spin else normalize(raw_angle)

            if not cmd.get('logged'):
                cmd['logged'] = True
                self.get_logger().info(f"turn cmd {math.degrees(angle):+.1f} deg (start yaw {math.degrees(start_yaw):+.1f})")

            T = duration_for_peak(angle, a.turn_speed)
            if T <= 0:
                self.current_cmd = None
                return

            homing = elapsed >= T
            tau = min(elapsed / T, 1.0)
            w_ff = 0.0 if homing else (angle / T) * velocity_factor(tau)

            turned = normalize(yaw - start_yaw)
            if abs(angle) > math.pi:
                if spin > 0 and turned < 0:
                    turned += 2 * math.pi
                elif spin < 0 and turned > 0:
                    turned -= 2 * math.pi

            expected = angle * position_factor(tau)
            error = expected - turned if abs(angle) > math.pi else normalize(expected - turned)

            w_correction = self.tracking_correction('turn', error, a.corr_gain, a.corr_damping_gain,
                                                    a.turn_speed, a.angle_tol)
            self.send(wz=w_ff + w_correction)

            if homing and abs(error) <= a.angle_tol and abs(self.vicon_wz) < 0.05:
                self.get_logger().info(f"landed {math.degrees(normalize(cmd['target_yaw'] - yaw)):+.2f} deg off")
                self.send(0.0, 0.0, 0.0)
                self.current_cmd = None
            elif homing and elapsed >= T + a.homing_timeout:
                self.get_logger().warn(f'turn residual {math.degrees(error):.2f} deg at timeout')
                self.current_cmd = None

        elif ctype == 'drive':
            dist = cmd['distance']
            T = duration_for_peak(dist, a.max_speed)
            if T <= 0:
                self.current_cmd = None
                return

            if not cmd.get('logged'):
                cmd['logged'] = True
                self.get_logger().info(f"drive cmd {dist:+.3f}m (start yaw {math.degrees(start_yaw):+.1f})")

            homing = elapsed >= T
            tau = min(elapsed / T, 1.0)
            v_ff = 0.0 if homing else (dist / T) * velocity_factor(tau)

            dx = x - start_x
            dy = y - start_y
            travelled = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
            lateral = -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)

            expected = dist * position_factor(tau)
            error = expected - travelled

            v_correction = self.tracking_correction('drive', error, a.corr_gain, a.corr_damping_gain,
                                                    a.max_speed, a.position_tol)
            heading_fix = clamp(
                HEADING_GAIN * normalize(start_yaw - yaw
                                         - clamp(LATERAL_GAIN * lateral, -LATERAL_CLAMP, LATERAL_CLAMP)),
                -HEADING_FIX_LIMIT * a.turn_speed, HEADING_FIX_LIMIT * a.turn_speed)
            self.send(vx=v_ff + v_correction, wz=heading_fix)

            if homing and abs(error) <= a.position_tol and abs(self.vicon_v) < 0.02:
                self.get_logger().info(f"landed {error:+.3f}m along, {lateral:+.3f}m lateral off")
                self.send(0.0, 0.0, 0.0)
                self.current_cmd = None
            elif elapsed >= T + a.homing_timeout:
                self.get_logger().warn(f'drive residual {error:.3f}m at timeout')
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

            vx_corr = self.tracking_correction('move_x', err_x, a.corr_gain, a.corr_damping_gain,
                                               a.max_speed, a.position_tol)
            vy_corr = self.tracking_correction('move_y', err_y, a.corr_gain, a.corr_damping_gain,
                                               a.max_speed, a.position_tol)

            vx_world = (dx / T) * vf + vx_corr
            vy_world = (dy / T) * vf + vy_corr
            vx_body, vy_body = world_to_body(vx_world, vy_world, yaw)
            heading_fix = clamp(HEADING_GAIN * normalize(start_yaw - yaw),
                                -HEADING_FIX_LIMIT * a.turn_speed, HEADING_FIX_LIMIT * a.turn_speed)

            self.send(vx=vx_body, vy=vy_body, wz=heading_fix)

            if homing and math.hypot(err_x, err_y) <= a.position_tol and abs(self.vicon_v) < 0.02:
                self.send(0.0, 0.0, 0.0)
                self.current_cmd = None
            elif elapsed >= T + a.homing_timeout:
                self.get_logger().warn(f'move_to residual {math.hypot(err_x, err_y):.3f}m at timeout')
                self.current_cmd = None

        elif ctype == 'return_home_diff':
            hx, hy, hyaw = self.home
            dist = math.hypot(hx - x, hy - y)
            retries = cmd.get('retries', 0)
            if dist > a.return_position_tol and retries < a.return_home_max_retries:
                target_angle = math.atan2(hy - y, hx - x)
                self.plan.extendleft([
                    {'type': 'return_home_diff', 'retries': retries + 1},
                    {'type': 'drive', 'distance': dist},
                    {'type': 'turn', 'target_yaw': target_angle},
                ])
            else:
                if dist > a.return_position_tol:
                    self.get_logger().warn(f'return_home_diff: giving up after {retries} attempts, residual distance {dist:.3f}m')
                self.plan.appendleft({'type': 'turn', 'target_yaw': hyaw})
            self.current_cmd = None


# setup scripts

def initialize_robot(node):
    # wait for vicon pose
    node.get_logger().info(f'waiting for {node.a.vicon_topic}')
    deadline = node.get_time_sec() + 10.0
    while rclpy.ok() and not node.have_pose and node.get_time_sec() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.have_pose:
        raise RuntimeError(f'no vicon pose on {node.a.vicon_topic} within 10s')

    # motor power service
    if node.motor_client.wait_for_service(timeout_sec=3.0):
        future = node.motor_client.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not (future.done() and future.result() is not None and future.result().success):
            node.get_logger().warn('motor power enable failed or timed out')
    else:
        node.get_logger().warn('motor power service not found')

    node.home = node.pose()
    node.intended_yaw = node.home[2]
    node.get_logger().info(f'home: x={node.home[0]:.3f} y={node.home[1]:.3f} '
                           f'yaw={math.degrees(node.home[2]):+.2f} deg')


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--platform', choices=['diff', 'omni'], default='diff')
    p.add_argument('--grid-size', type=int, default=4)
    p.add_argument('--cell-size', type=float, default=0.25)
    p.add_argument('--max-speed', type=float, default=0.1)
    p.add_argument('--turn-speed', type=float, default=0.25)
    p.add_argument('--corr-gain', type=float, default=1.5)
    p.add_argument('--corr-damping-gain', type=float, default=0.3)
    p.add_argument('--corr-filter-tau', type=float, default=0.08)
    p.add_argument('--position-tol', type=float, default=0.005)
    p.add_argument('--return-position-tol', type=float, default=0.01)
    p.add_argument('--angle-tol', type=float, default=0.0087)
    p.add_argument('--rate', type=float, default=25.0)
    p.add_argument('--homing-timeout', type=float, default=3.0)
    p.add_argument('--return-home-max-retries', type=int, default=8)
    p.add_argument('--max-accel', type=float, default=0.3)
    p.add_argument('--max-ang-accel', type=float, default=0.5)
    p.add_argument('--no-return-home', action='store_true')
    p.add_argument('--cmd-vel-topic', default='/cmd_vel')
    p.add_argument('--vicon-topic', default='/vicon/aurora3_robot/aurora3_robot')
    p.add_argument('--vicon-timeout', type=float, default=0.5)
    p.add_argument('--vel-filter-tau', type=float, default=0.05)
    p.add_argument('--motor-power-service', default='/motor_power')
    p.add_argument('--corr-max-d-error', type=float, default=1.0)
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
            try:
                node.send(0.0, 0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.02)
            except Exception:
                break
            time.sleep(0.02)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
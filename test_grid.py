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


def ask_grid_size(default_n):
    if not sys.stdin.isatty():
        return default_n
    while True:
        raw = input(f'grid size N for an N x N grid [{default_n}]: ').strip()
        if raw == '':
            return default_n
        try:
            n = int(raw)
        except ValueError:
            print('enter a whole number')
            continue
        if n < 1:
            print('must be at least 1')
            continue
        return n


def ask_total_size():
    if not sys.stdin.isatty():
        return None
    raw = input('square size in meters or blank to set cell size directly: ').strip()
    if raw == '':
        return None
    try:
        size = float(raw)
    except ValueError:
        return None
    if size <= 0:
        return None
    return size


class GridMotionNode(Node):
    def __init__(self, args):
        super().__init__('grid_motion_node')
        self.a = args
        self.x = self.y = self.yaw = 0.0
        self.have_odom = False
        self.home = None

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, qos)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos)
        self.motor_client = self.create_client(SetBool, args.motor_power_service)
        self.reset_client = self.create_client(Trigger, args.reset_odom_service)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def pose(self):
        return self.x, self.y, self.yaw

    def enable_motors(self):
        if not self.motor_client.wait_for_service(timeout_sec=3.0):
            print('motor power service not found, skipping')
            return
        req = SetBool.Request()
        req.data = True
        future = self.motor_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print('motor power enable result:', future.result())

    def reset_odom(self):
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            print('reset odom not found')
            return
        future = self.reset_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        print('reset odom result:', future.result())

    def wait_for_odom(self, timeout=10.0):
        print('waiting for /odom')
        start = time.monotonic()
        while rclpy.ok() and not self.have_odom:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > timeout:
                raise RuntimeError('no odom data')
        print('got /odom')

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.pub.publish(msg)

    def stop(self, duration=None):
        duration = self.a.settle_delay if duration is None else duration
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            self.send()
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)

    def turn(self, target_yaw, max_speed=None):
        a = self.a
        max_speed = a.turn_speed if max_speed is None else max_speed
        _, _, start_yaw = self.pose()
        angle = normalize(target_yaw - start_yaw)
        T = duration_for_peak(angle, max_speed)
        if T <= 0:
            return

        direction = 'left' if angle > 0 else 'right'
        print(f'turning {direction} {math.degrees(abs(angle)):.1f} deg over {T:.2f}s')

        move_start = time.monotonic()
        while rclpy.ok():
            elapsed = time.monotonic() - move_start
            if elapsed >= T:
                break
            tau = elapsed / T
            w_ff = (angle / T) * velocity_factor(tau)
            rclpy.spin_once(self, timeout_sec=0.0)
            _, _, yaw = self.pose()
            turned = normalize(yaw - start_yaw)
            expected = angle * position_factor(tau)
            w_correction = clamp(a.corr_gain * (expected - turned), -0.3 * max_speed, 0.3 * max_speed)
            self.send(wz=w_ff + w_correction)
            time.sleep(1.0 / a.rate)
        self.stop(0.05)

        # small correction
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            _, _, yaw = self.pose()
            error = normalize(target_yaw - yaw)
            if abs(error) <= a.angle_tol:
                break
            self.send(wz=math.copysign(clamp(1.25 * abs(error), 0.0, max_speed), error))
            time.sleep(1.0 / a.rate)
        self.stop()

    def turn_by(self, angle, max_speed=None):
        _, _, yaw = self.pose()
        self.turn(normalize(yaw + angle), max_speed)

    def drive(self, distance, max_speed=None):
        if distance <= 0:
            return
        a = self.a
        max_speed = a.max_speed if max_speed is None else max_speed
        T = duration_for_peak(distance, max_speed)
        if T <= 0:
            return

        print(f'driving forward {distance:.3f}m over {T:.2f}s')
        start_x, start_y, start_yaw = self.pose()
        move_start = time.monotonic()
        while rclpy.ok():
            elapsed = time.monotonic() - move_start
            if elapsed >= T:
                break
            tau = elapsed / T
            v_ff = (distance / T) * velocity_factor(tau)
            # process odometry
            rclpy.spin_once(self, timeout_sec=0.0)
            x, y, yaw = self.pose()
            travelled = math.hypot(x - start_x, y - start_y)
            expected = distance * position_factor(tau)
            v_correction = clamp(a.corr_gain * (expected - travelled), -0.3 * max_speed, 0.3 * max_speed)
            heading_fix = clamp(1.8 * normalize(start_yaw - yaw), -0.5 * a.turn_speed, 0.5 * a.turn_speed)
            self.send(vx=v_ff + v_correction, wz=heading_fix)
            time.sleep(1.0 / a.rate)
        self.stop(0.05)

        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            x, y, yaw = self.pose()
            remaining = distance - math.hypot(x - start_x, y - start_y)
            if remaining <= a.position_tol:
                break
            heading_fix = clamp(1.8 * normalize(start_yaw - yaw), -0.5 * a.turn_speed, 0.5 * a.turn_speed)
            self.send(vx=clamp(remaining, 0.0, max_speed), wz=heading_fix)
            time.sleep(1.0 / a.rate)
        self.stop()

    def go_home(self):
        hx, hy, hyaw = self.home
        a = self.a
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            x, y, _ = self.pose()
            dist = math.hypot(hx - x, hy - y)
            if dist <= a.position_tol:
                break
            self.turn(math.atan2(hy - y, hx - x), max_speed=a.turn_speed)
            self.drive(dist, max_speed=a.max_speed)
        print('restoring original orientation')
        self.turn(hyaw, max_speed=a.turn_speed)

    def drive_grid(self):
        n = self.a.grid_size
        cell = self.a.cell_size
        print(f'starting {n}x{n} grid, cell size {cell:.3f} m')
        if n <= 1:
            print('grid size 1')
            return
        for row in range(n):
            print(f'row {row + 1}/{n}')
            for _ in range(n - 1):
                self.drive(cell)
            if row < n - 1:
                turn = -math.pi / 2 if row % 2 == 0 else math.pi / 2
                self.turn_by(turn)
                self.drive(cell)
                self.turn_by(turn)

    def run(self):
        self.enable_motors()
        self.reset_odom()
        self.wait_for_odom()
        self.stop(0.3)

        self.home = self.pose()
        hx, hy, hyaw = self.home
        print(f'home pose: x={hx:.3f} y={hy:.3f} yaw={math.degrees(hyaw):.1f}deg')

        self.drive_grid()

        if not self.a.no_return_home:
            print('going back to start')
            self.go_home()

        self.stop(0.8)
        print('grid complete')


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--grid-size', type=int, default=None)
    p.add_argument('--no-prompt', action='store_true')
    p.add_argument('--cell-size', type=float, default=None)
    p.add_argument('--total-size', type=float, default=None)
    p.add_argument('--max-speed', type=float, default=0.05)
    p.add_argument('--turn-speed', type=float, default=0.25)
    p.add_argument('--corr-gain', type=float, default=1.5)
    p.add_argument('--position-tol', type=float, default=0.01)
    p.add_argument('--angle-tol', type=float, default=0.025)
    p.add_argument('--rate', type=float, default=25.0)
    p.add_argument('--settle-delay', type=float, default=0.3)
    p.add_argument('--no-return-home', action='store_true')
    p.add_argument('--cmd-vel-topic', default='/cmd_vel')
    p.add_argument('--motor-power-service', default='/motor_power')
    p.add_argument('--reset-odom-service', default='/reset_odometry')

    args = p.parse_args()

    if args.grid_size is None:
        args.grid_size = 4 if args.no_prompt else ask_grid_size(4)

    if args.total_size is None and args.cell_size is None and not args.no_prompt and args.grid_size >= 2:
        args.total_size = ask_total_size()
    if args.total_size is not None and args.cell_size is not None:
        p.error('use either --cell-size or --total-size not both')
    if args.total_size is not None:
        args.cell_size = args.total_size / max(args.grid_size - 1, 1)
    elif args.cell_size is None:
        args.cell_size = 0.25

    return args


def main():
    args = parse_args()
    rclpy.init()
    node = GridMotionNode(args)
    try:
        node.run()
    except KeyboardInterrupt:
        print('stopping robot')
        node.stop(1.0)
    except Exception as exc:
        print('error:', exc)
        node.stop(1.0)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
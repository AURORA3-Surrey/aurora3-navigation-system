import argparse
import json
import math
import os
import signal
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool


def normalize(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class SafetyAbort(RuntimeError):
    pass


class YawCalibNode(Node):
    # measures the yaw offset by moving forward and watching which world direction the robot moves
    # (measured offset only used for the return trip)

    def __init__(self, args):
        super().__init__('vicon_yaw_calibration')
        self.a = args
        self.x = self.y = self.yaw = 0.0
        self.have_pose = False
        self.last_pose_time = None
        self.off_rad = 0.0  # measured offset set after movement

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, qos)
        self.create_subscription(PoseStamped, args.vicon_topic, self.vicon_cb, qos)
        self.motor_client = self.create_client(SetBool, args.motor_power_service)

        self.last_vx = self.last_wz = 0.0
        self.last_send_time = None
        self.timer_period = 1.0 / self.a.rate

    def vicon_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.orientation)  # no offset
        self.have_pose = True
        self.last_pose_time = time.time()

    def pose(self):
        return self.x, self.y, self.yaw

    def get_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def send(self, vx=0.0, wz=0.0):
        now = self.get_time_sec()
        dt = (now - self.last_send_time) if self.last_send_time else self.timer_period
        self.last_send_time = now
        max_dv = self.a.accel_lin * dt
        max_dw = self.a.accel_ang * dt
        vx = clamp(vx, self.last_vx - max_dv, self.last_vx + max_dv)
        wz = clamp(wz, self.last_wz - max_dw, self.last_wz + max_dw)
        self.last_vx, self.last_wz = vx, wz

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.angular.z = float(wz)
        self.pub.publish(msg)

    def spin_tick(self):
        rclpy.spin_once(self, timeout_sec=0.005)
        if self.last_pose_time is not None and time.time() - self.last_pose_time > self.a.vicon_timeout:
            raise SafetyAbort(f'vicon stream stale for {time.time() - self.last_pose_time:.1f}s')


def wait_for_pose(node, timeout=10.0):
    node.get_logger().info(f'waiting for {node.a.vicon_topic}')
    t0 = time.time()
    while not node.have_pose:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > timeout:
            raise SafetyAbort('no vicon pose received')
    node.get_logger().info('vicon stream live')


def wait_stationary(node, timeout=10.0):
    # start pose must be still
    node.get_logger().info('waiting for robot to be stationary')
    t0 = time.time()
    while time.time() - t0 < timeout:
        samples = []
        while len(samples) < 15 and time.time() - t0 < timeout:
            node.spin_tick()
            samples.append(node.pose())
            time.sleep(0.03)
        xs = [s[0] for s in samples]
        ys = [s[1] for s in samples]
        spread = max(max(xs) - min(xs), max(ys) - min(ys))
        if spread < 0.005:
            return
    raise SafetyAbort('robot not stationary aborting')


def set_motor_power(node, on):
    if not node.motor_client.wait_for_service(timeout_sec=3.0):
        node.get_logger().warn('motor_power service not available continuing')
        return False
    fut = node.motor_client.call_async(SetBool.Request(data=on))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    ok = fut.done() and fut.result() is not None and fut.result().success
    node.get_logger().info(f'motor power {"ON" if on else "OFF"} {ok}')
    return ok


def zero_burst(node, n=10):
    sent = 0
    for _ in range(n):
        try:
            node.send(0.0, 0.0)
            sent += 1
            rclpy.spin_once(node, timeout_sec=0.01)
        except Exception:
            break
        time.sleep(0.02)
    node.get_logger().info(f'stop: {sent}/{n} zero commands published')


def move_forward(node):
    a = node.a
    x0, y0, yaw0 = node.pose()
    node.get_logger().info(
        f'start pose: ({x0:+.3f}, {y0:+.3f}) raw vicon yaw {math.degrees(yaw0):+.1f} deg')
    node.get_logger().info(
        f'forward movement {a.movement_distance:.2f} m at {a.movement_speed:.2f} m/s')
    t0 = time.time()
    while time.time() - t0 < a.movement_timeout:
        node.spin_tick()
        x, y, yaw = node.pose()
        disp = math.hypot(x - x0, y - y0)
        if disp >= a.movement_distance:
            break
        if disp > a.max_displacement:
            raise SafetyAbort(f'displacement runaway: {disp:.3f} m')
        wz = clamp(1.5 * normalize(yaw0 - yaw), -0.3, 0.3)
        node.send(vx=a.movement_speed, wz=wz)
        time.sleep(1.0 / a.rate)
    node.send(0.0, 0.0)
    x, y, yaw = node.pose()
    disp = math.hypot(x - x0, y - y0)
    if disp < a.min_displacement:
        raise SafetyAbort(f'robot moved only {disp:.3f} m (< {a.min_displacement:.2f} m min) motors enabled?')
    return x0, y0, yaw0, x, y, yaw, disp


def measure_offset(x0, y0, yaw0, x, y, yaw_end):
    bearing = math.atan2(y - y0, x - x0)
    yaw_mean = yaw0 + 0.5 * normalize(yaw_end - yaw0)
    off = math.degrees(normalize(bearing - yaw_mean))
    snap = round(off / 90.0) * 90.0
    snap = ((snap + 180.0) % 360.0) - 180.0  # [-180, 180]
    if snap == -180.0:
        snap = 180.0
    residual = math.degrees(normalize(math.radians(off - snap)))
    return bearing, yaw_mean, off, snap, residual


def return_home(node, x0, y0, yaw0):
    # drive back to the start pose using the measured offset (+ restore original heading)
    a = node.a
    off = node.off_rad
    node.get_logger().info('turn toward start, drive back, restore heading')

    t0 = time.time()
    while time.time() - t0 < a.return_timeout:
        node.spin_tick()
        x, y, yaw = node.pose()
        dist = math.hypot(x0 - x, y0 - y)
        if dist <= a.home_tol:
            break
        err = normalize(math.atan2(y0 - y, x0 - x) - (yaw + off))
        v = clamp(1.5 * dist, 0.0, a.return_speed) if abs(err) <= math.radians(30.0) else 0.0
        node.send(vx=v, wz=clamp(2.0 * err, -a.turn_speed, a.turn_speed))
        time.sleep(1.0 / a.rate)
    node.send(0.0, 0.0)

    # restore original heading
    t0 = time.time()
    while time.time() - t0 < a.turn_timeout:
        node.spin_tick()
        _, _, yaw = node.pose()
        err = normalize(yaw0 - yaw)
        if abs(err) <= math.radians(a.turn_tol_deg):
            break
        node.send(wz=clamp(2.0 * err, -a.turn_speed, a.turn_speed))
        time.sleep(1.0 / a.rate)
    node.send(0.0, 0.0)
    return node.pose()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--vicon-topic', default='/vicon/Turtlebot3/Turtlebot3')
    p.add_argument('--cmd-vel-topic', default='/cmd_vel')
    p.add_argument('--motor-power-service', default='/motor_power')
    p.add_argument('--movement-speed', type=float, default=0.05)
    p.add_argument('--movement-distance', type=float, default=0.12)
    p.add_argument('--min-displacement', type=float, default=0.03)
    p.add_argument('--max-displacement', type=float, default=0.30)
    p.add_argument('--movement-timeout', type=float, default=10.0)
    p.add_argument('--return-speed', type=float, default=0.06)
    p.add_argument('--turn-speed', type=float, default=0.4)
    p.add_argument('--turn-tol-deg', type=float, default=2.0)
    p.add_argument('--return-timeout', type=float, default=25.0)
    p.add_argument('--turn-timeout', type=float, default=8.0)
    p.add_argument('--home-tol', type=float, default=0.015)
    p.add_argument('--vicon-timeout', type=float, default=0.5)
    p.add_argument('--rate', type=float, default=25.0)
    p.add_argument('--accel-lin', type=float, default=0.3)
    p.add_argument('--accel-ang', type=float, default=1.0)
    p.add_argument('--keep-motors-on', action='store_true')
    p.add_argument('--no-save', action='store_true')
    p.add_argument('--save-file', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vicon_yaw_offset.json'))
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = YawCalibNode(args)

    def stop_on_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_on_sigterm)

    exit_code = 1
    try:
        wait_for_pose(node)
        wait_stationary(node)
        set_motor_power(node, True)
        zero_burst(node, 5)

        x0, y0, yaw0, xe, ye, yawe, disp = move_forward(node)
        bearing, yaw_mean, off_deg, snap, residual = measure_offset(x0, y0, yaw0, xe, ye, yawe)
        node.off_rad = math.radians(off_deg)

        node.get_logger().info(
            f'movement: {disp:.3f} m, world bearing {math.degrees(bearing):+.1f} deg, '
            f'raw vicon yaw mean {math.degrees(yaw_mean):+.1f} deg')
        node.get_logger().info(
            f'MEASURED yaw offset: {off_deg:+.1f} deg snapped {snap:+.0f} deg '
            f'(residual {residual:+.1f} deg)')
        if abs(residual) > 20.0:
            node.get_logger().warn('residual > 20 deg: movement was not clean')

        if snap == 0.0:
            node.get_logger().info('RESULT: Vicon +x matches robot forward (no offset needed)')
        elif snap == 180.0:
            node.get_logger().info('RESULT: Vicon +x faces the robot REAR')
        else:
            node.get_logger().warn('RESULT: rigid body is rotated around 90 deg, recreate the rigid body in Tracker')

        node.get_logger().info(f'RECOMMENDED: --vicon-yaw-offset-deg {snap:+.0f}')
        node.get_logger().info(f'YAW_OFFSET_RESULT: {snap:+.0f}')

        fx, fy, fyaw = return_home(node, x0, y0, yaw0)
        dx, dy = fx - x0, fy - y0
        dyaw = math.degrees(normalize(fyaw - yaw0))
        dist = math.hypot(dx, dy)
        node.get_logger().info(
            f'RETURN: at ({fx:+.3f}, {fy:+.3f}) | dx={dx:+.3f}m dy={dy:+.3f}m '
            f'dyaw={dyaw:+.2f} deg')
        returned = dist <= 0.04 and abs(dyaw) <= 5.0
        if returned:
            node.get_logger().info('RETURN OK measured offset validated')
        else:
            node.get_logger().warn('return imperfect reposition the robot by hand (measurement still valid)')

        if not args.no_save:
            payload = {
                'offset_deg': snap,
                'measured_deg': round(off_deg, 1),
                'snap_residual_deg': round(residual, 1),
                'vicon_topic': args.vicon_topic,
                'created': time.strftime('%Y-%m-%d %H:%M:%S'),
            }
            with open(args.save_file, 'w') as f:
                json.dump(payload, f, indent=2)
            node.get_logger().info(
                f'saved: {args.save_file} (vicon_grid.py auto-load)')

        exit_code = 0 if returned else 2

    except SafetyAbort as exc:
        node.get_logger().error(f'ABORT: {exc}')
        exit_code = 1
    except KeyboardInterrupt:
        node.get_logger().warn('stopping robot')
        exit_code = 1
    except Exception as exc:
        node.get_logger().error(f'error: {exc}')
        raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.last_vx = node.last_wz = 0.0
        zero_burst(node, 10)
        if not args.keep_motors_on:
            try:
                set_motor_power(node, False)
            except Exception:
                pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
import argparse
import rclpy
import math
import time
import sys


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ViconCheck(Node):
    def __init__(self, args):
        super().__init__('vicon_check')
        self.a = args
        self.count = self.bad_pose = self.bad_quat = self.bad_stamps = 0
        self.first_recv = self.last_recv = self.prev_recv = self.prev_stamp = None
        self.first_pos = self.last_pos = None
        self.last_yaw = 0.0
        self.frame_id = None
        self.intervals = 0
        self.stamp_sum = self.arrival_sum = 0.0
        self.max_stamp_gap = self.max_arrival_gap = self.max_lag = self.max_quat_error = 0.0
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseStamped, args.vicon_topic, self.cb, qos)

    def cb(self, msg):
        local_recv_time = time.time()
        recv = time.monotonic()
        p, q = msg.pose.position, msg.pose.orientation
        x, y = p.x, p.y
        yaw = yaw_from_quaternion(q)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not all(math.isfinite(v) for v in (x, y, yaw)):
            self.bad_pose += 1

        qn = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        qerr = abs(qn - 1.0) if math.isfinite(qn) else float('inf')
        self.max_quat_error = max(self.max_quat_error, qerr)
        if qerr > 0.01:
            self.bad_quat += 1

        if self.prev_recv is None:
            self.first_recv, self.first_pos = recv, (x, y)
        else:
            arrival_dt = recv - self.prev_recv
            stamp_dt = stamp - self.prev_stamp
            self.intervals += 1
            self.arrival_sum += arrival_dt
            self.stamp_sum += stamp_dt
            self.max_arrival_gap = max(self.max_arrival_gap, arrival_dt)
            if stamp_dt <= 0:
                self.bad_stamps += 1
            else:
                self.max_stamp_gap = max(self.max_stamp_gap, stamp_dt)

        self.max_lag = max(self.max_lag, abs(local_recv_time - stamp))
        self.prev_recv = self.last_recv = recv
        self.prev_stamp = stamp
        self.last_pos, self.last_yaw = (x, y), yaw
        self.frame_id = msg.header.frame_id
        self.count += 1

    def report(self):
        print(f'\nVicon: {self.a.vicon_topic} | frame: {self.frame_id} | messages: {self.count}')
        if not self.count:
            print('FAIL: no messages received\n')
            return False

        elapsed = self.last_recv - self.first_recv
        rate = (self.count - 1) / elapsed if elapsed > 0 else 0.0
        arrival_mean = self.arrival_sum / self.intervals if self.intervals else 0.0
        stamp_mean = self.stamp_sum / self.intervals if self.intervals else 0.0
        silence = time.monotonic() - self.last_recv
        arrival_gap = max(self.max_arrival_gap, silence)
        displacement = math.dist(self.first_pos, self.last_pos)

        print(f'Rate: {rate:.1f} Hz | arrival: {arrival_mean * 1000:.1f} ms mean, {self.max_arrival_gap * 1000:.1f} ms max')
        print(f'Stamps: {stamp_mean * 1000:.1f} ms mean, {self.max_stamp_gap * 1000:.1f} ms max positive')
        print(f'Position: ({self.last_pos[0]:+.3f}, {self.last_pos[1]:+.3f}) m | yaw: {math.degrees(self.last_yaw):+.1f} deg')
        print(f'Displacement: {displacement * 1000:.1f} mm')

        checks = [
            (rate >= self.a.min_rate, f'rate below {self.a.min_rate:.1f} Hz', f'rate above {self.a.min_rate:.1f} Hz'),
            (self.intervals > 0 and self.bad_stamps == 0 and self.max_stamp_gap <= 0.2, f'timestamp check failed: {self.bad_stamps} non-increasing intervals, max gap {self.max_stamp_gap:.3f} s', 'timestamps increase and gaps are within 0.2 s'),
            (self.intervals > 0 and arrival_gap <= 0.5, f'arrival check failed: {self.intervals} intervals, gap or silence {arrival_gap:.3f} s', 'message arrivals within limit'),
            (self.bad_pose == 0, f'{self.bad_pose} invalid positions or yaw values', 'positions and yaw are finite'),
            (self.bad_quat == 0, f'{self.bad_quat} invalid quaternions; max norm error {self.max_quat_error:.4f}', 'quaternion norms within tolerance'),
        ]
        failures = [f for passed, f, p in checks if not passed]
        passes = [p for passed, f, p in checks if passed]
        warnings = []

        if self.max_lag > 1.0:
            warnings.append(f'timestamp differs from local clock by up to {self.max_lag:.2f} s')
        if self.a.expect_motion and displacement < 0.05:
            failures.append('expected movement but displacement was below 50 mm')
        elif self.a.expect_motion:
            passes.append('movement detected')
        elif displacement < 0.001:
            warnings.append('pose remained static; movement was not tested')

        for title, items, prefix in [('FAILURES', failures, 'FAIL'), ('PASSES', passes, 'PASS'), ('WARNINGS', warnings, 'WARN')]:
            print(f'\n{title}')
            for item in items:
                print(f'{prefix}: {item}')
            if not items:
                print('none')

        if failures:
            print('\nRESULT: FAIL')
            return False
        print('\nRESULT: PASS')
        return True
    

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--vicon-topic', default='/vicon/Dingo/Dingo')
    p.add_argument('--duration', type=float, default=10.0)
    p.add_argument('--min-rate', type=float, default=25.0)
    p.add_argument('--expect-motion', action='store_true')
    args = p.parse_args()

    rclpy.init()
    node = ViconCheck(args)
    ok = False
    try:
        end = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        ok = node.report()
    except KeyboardInterrupt:
        ok = node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
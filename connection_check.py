import argparse
import math
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ViconCheck(Node):
    # listens to the vicon topic and reports rate, stamp health and data sanity
    def __init__(self, args):
        super().__init__('vicon_check')
        self.a = args
        self.count = 0
        self.first_recv = None
        self.last_recv = None
        self.prev = None              # (x, y, yaw, stamp, recv_time)
        self.stamp_dts = []           # deltas between consecutive header stamps
        self.recv_dts = []            # deltas between consecutive arrivals
        self.lags = []                # arrival time - header stamp
        self.distinct_stamps = set()
        self.first_pos = None
        self.last_pos = None
        self.last_yaw = None
        self.max_coord = 0.0
        self.nonfinite = 0
        self.bad_quat = 0
        self.frame_id = None

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseStamped, args.vicon_topic, self.cb, qos)
        self.get_logger().info(f'listening on {args.vicon_topic} for {args.duration:.0f}s')

    def cb(self, msg):
        recv = self.get_clock().now().nanoseconds / 1e9
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            self.nonfinite += 1
        q = msg.pose.orientation
        qn = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if abs(qn - 1.0) > 0.01:
            self.bad_quat += 1

        if self.first_recv is None:
            self.first_recv = recv
            self.first_pos = (x, y)
        if self.prev is not None:
            self.recv_dts.append(recv - self.prev[4])
            self.stamp_dts.append(stamp - self.prev[3])
        self.lags.append(recv - stamp)
        self.distinct_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))

        self.frame_id = msg.header.frame_id
        self.max_coord = max(self.max_coord, abs(x), abs(y))
        self.last_pos = (x, y)
        self.last_yaw = yaw
        self.prev = (x, y, yaw, stamp, recv)
        self.count += 1

    def stats(self, values):
        if not values:
            return 0.0, 0.0
        return sum(values) / len(values), max(values)

    def report(self):
        a = self.a
        print('\nvicon connectivity')
        print(f'topic: {a.vicon_topic}')
        print(f'frame_id: {self.frame_id}')
        print(f'messages: {self.count}')

        if self.count == 0:
            print('\nFAIL: no messages received')
            return False

        elapsed = self.last_recv - self.first_recv
        rate = (self.count - 1) / elapsed if elapsed > 0 else 0.0
        lag_mean, lag_max = self.stats(self.lags)
        sdt_mean, sdt_max = self.stats(self.stamp_dts)
        rdt_mean, rdt_max = self.stats(self.recv_dts)

        print(f'rate: {rate:.1f} Hz (over {elapsed:.1f}s)')
        print(f'arrival dt: mean {rdt_mean * 1000:.1f} ms, max {rdt_max * 1000:.1f} ms')
        print(f'stamp dt: mean {sdt_mean * 1000:.1f} ms, max {sdt_max * 1000:.1f} ms')
        print(f'stamp lag: mean {lag_mean * 1000:+.1f} ms, max {lag_max * 1000:+.1f} ms (arrival - header stamp)')
        print(f'position: x={self.last_pos[0]:+.3f} y={self.last_pos[1]:+.3f} m, yaw={math.degrees(self.last_yaw):+.2f} deg')
        print(f'displacement: {math.hypot(self.last_pos[0] - self.first_pos[0], self.last_pos[1] - self.first_pos[1]) * 1000:.1f} mm over window')

        ok = True

        if rate < a.min_rate:
            print(f'FAIL: rate {rate:.1f} Hz below minimum {a.min_rate:.1f} Hz')
            ok = False
        else:
            print(f'OK: rate above {a.min_rate:.1f} Hz floor')

        if len(self.distinct_stamps) <= 1:
            print('FAIL: header.stamp never changes')
            ok = False
        else:
            print(f'OK: stamps advancing ({len(self.distinct_stamps)} distinct stamps)')

        if any(d <= 0 for d in self.stamp_dts):
            print('WARN: header.stamp went backwards')
        if rdt_max > 0.5:
            print(f'WARN: stream stalled {rdt_max:.2f}s; aborts at 0.5s of silence')
        if sdt_max > 0.2:
            print(f'INFO: stamp gaps above 0.2s zero the velocity estimate momentarily (matters while moving)')

        if abs(lag_max) > 1.0:
            print(f'INFO: stamp lag up to {lag_max:+.2f}s is just bridge-PC vs robot clock offset')

        if self.nonfinite:
            print(f'FAIL: {self.nonfinite} messages with non-finite position/yaw')
            ok = False
        if self.bad_quat:
            print(f'WARN: {self.bad_quat} messages with non-normalized quaternion (|{abs(1.0 - 1.0):.0f}| norm off by >0.01)')
        if self.max_coord > 100.0:
            print(f'WARN: coordinates reach {self.max_coord:.1f} (expected meters check bridge units)')

        disp = math.hypot(self.last_pos[0] - self.first_pos[0], self.last_pos[1] - self.first_pos[1])
        if a.expect_motion:
            if disp < 0.05:
                print('FAIL: --expect-motion set but displacement less than 50 mm (pose frozen or robot not moved)')
                ok = False
            else:
                print('OK: live tracking confirmed (displacement above 50 mm)')
        elif disp < 0.001:
            print('INFO: pose is static to confirm live tracking, wiggle the robot and re-run with --expect-motion')

        print('\nRESULT: PASS' if ok else '\nRESULT: FAIL')
        return ok


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--vicon-topic', default='/vicon/aurora3_robot/aurora3_robot')
    p.add_argument('--duration', type=float, default=10.0) # seconds of test window
    p.add_argument('--min-rate', type=float, default=10.0) # mium accepted mess rate [Hz]
    p.add_argument('--expect-motion', action='store_true') # fails unless the pose moves over 50mm during window
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ViconCheck(args)
    t0 = time.monotonic()
    next_status = 1.0
    prev_count = 0
    ok = False
    try:
        while rclpy.ok() and time.monotonic() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            t = time.monotonic() - t0
            if t >= next_status:
                next_status += 1.0
                live_rate = node.count - prev_count
                prev_count = node.count
                if node.count:
                    x, y = node.last_pos
                    print(f' t={t:3.0f}s msgs={node.count} rate={live_rate:.0f}/s '
                          f'pos=({x:+.3f}, {y:+.3f})m yaw={math.degrees(node.last_yaw):+.1f} deg')
                else:
                    print(f't={t:3.0f}s no messages yet')
        ok = node.report()
    except KeyboardInterrupt:
        ok = node.report()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
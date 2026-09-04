import argparse
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


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


class ViconToOdom(Node):
    # bridges geometry_msgs/PoseStamped (vicon) -> nav_msgs/Odometry
    def __init__(self, args):
        super().__init__('vicon_to_odom')
        self.a = args
        self.prev = None   # (x, y, yaw, stamp) of previous vicon sample
        self.v = 0.0
        self.wz = 0.0
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseStamped, args.vicon_topic, self.vicon_cb, qos)
        self.pub = self.create_publisher(Odometry, args.odom_topic, qos)
        self.get_logger().info(f'{args.vicon_topic} -> {args.odom_topic}')

    def vicon_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        prev = self.prev
        if prev is not None and 0.0 < stamp - prev[3] < 0.2:
            dt = stamp - prev[3]
            v_raw = math.hypot(x - prev[0], y - prev[1]) / dt
            w_raw = normalize(yaw - prev[2]) / dt
            alpha = dt / (self.a.vel_filter_tau + dt)
            self.v = alpha * v_raw + (1.0 - alpha) * self.v
            self.wz = alpha * w_raw + (1.0 - alpha) * self.wz
        else:
            self.v = 0.0
            self.wz = 0.0
        self.prev = (x, y, yaw, stamp)

        # optional rigid-body alignment (uniform scale + yaw offset about world Z)
        s = self.a.scale
        off = math.radians(self.a.yaw_offset_deg)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = 'base_footprint'
        if off != 0.0:
            c, si = math.cos(off), math.sin(off)
            out.pose.pose.position.x = s * (c * x - si * y)
            out.pose.pose.position.y = s * (si * x + c * y)
            yaw_out = normalize(yaw + off)
            out.pose.pose.orientation.z = math.sin(yaw_out / 2.0)
            out.pose.pose.orientation.w = math.cos(yaw_out / 2.0)
        else:
            out.pose.pose.position.x = s * x
            out.pose.pose.position.y = s * y
            out.pose.pose.orientation = msg.pose.orientation
        out.twist.twist.linear.x = self.v
        out.twist.twist.angular.z = self.wz
        self.pub.publish(out)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--vicon-topic', default='/vicon/aurora3_robot/aurora3_robot')
    p.add_argument('--odom-topic', default='/vicon/odom')
    p.add_argument('--vel-filter-tau', type=float, default=0.05)
    p.add_argument('--yaw-offset-deg', type=float, default=0.0)
    p.add_argument('--scale', type=float, default=1.0)
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ViconToOdom(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
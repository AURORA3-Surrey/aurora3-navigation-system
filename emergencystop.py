import argparse
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Stop(Node):
    # zero cmd_vel burst (run this from a second SSH window)
    def __init__(self, args):
        super().__init__('stop')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, qos)
        self.client = self.create_client(SetBool, args.motor_power_service)
        self.get_logger().info(f'stop armed: {args.cmd_vel_topic} + {args.motor_power_service}')

    def send_zero(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.angular.z = 0.0
        self.pub.publish(msg)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cmd-vel-topic', default='/cmd_vel')
    p.add_argument('--motor-power-service', default='/motor_power')
    p.add_argument('--bursts', type=int, default=20)
    p.add_argument('--keep-motors-on', action='store_true')
    args = p.parse_args()

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    node = Stop(args)
    sent = 0
    try:
        try:
            for _ in range(args.bursts):
                node.send_zero()
                sent += 1
                rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(0.02)
        except KeyboardInterrupt:
            node.get_logger().warn(
                f'stop: interrupted after {sent}/{args.bursts} zeros, cutting motor power')
        node.get_logger().info(f'stop: {sent}/{args.bursts} zero commands published on {args.cmd_vel_topic}')

        if args.keep_motors_on:
            node.get_logger().info('stop: motors enabled (--keep-motors-on)')
        elif node.client.wait_for_service(timeout_sec=2.0):
            future = node.client.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            if future.done() and future.result() is not None and future.result().success:
                node.get_logger().info('stop: motor power OFF')
            else:
                node.get_logger().warn('stop: motor power off failed')
        else:
            node.get_logger().warn('stop: /motor_power service not found')
    except KeyboardInterrupt:
        node.get_logger().warn('stop: interrupted, motor power state unconfirmed')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

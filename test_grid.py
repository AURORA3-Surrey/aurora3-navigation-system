import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool, Trigger

SIDE_LENGTH = 0.3   # m
SPEED = 0.05        # m/s
TURN_SPEED = 0.3    # rad/s


class SquareMotionNode(Node):
    def __init__(self):
        super().__init__('square_motion_node')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.motor_client = self.create_client(SetBool, '/motor_power')
        self.reset_client = self.create_client(Trigger, '/reset_odometry')

    def enable_motors(self):
        self.motor_client.wait_for_service()
        req = SetBool.Request()
        req.data = True
        future = self.motor_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def reset_odom(self):
        self.reset_client.wait_for_service()
        future = self.reset_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

    def send(self, linear, angular, duration):
        end = time.time() + duration
        while time.time() < end:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = linear
            msg.twist.angular.z = angular
            self.pub.publish(msg)
            time.sleep(0.1)
        self.stop()

    def stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        time.sleep(0.2)

    def run(self):
        self.enable_motors()
        self.reset_odom()
        forward_time = SIDE_LENGTH / SPEED
        turn_time = 1.5708 / TURN_SPEED  # 90 deg
        for _ in range(4):
            self.send(SPEED, 0.0, forward_time)
            self.send(0.0, TURN_SPEED, turn_time)


def main():
    rclpy.init()
    node = SquareMotionNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
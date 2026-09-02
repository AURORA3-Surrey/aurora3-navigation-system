import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger

SIDE_LENGTH = 0.3   # m
SPEED = 0.05        # m/s
TURN_SPEED = 0.3    # rad/s
POSITION_TOL = 0.01 # m
ANGLE_TOL = 0.02    # rad


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


class SquareMotionNode(Node):
    def __init__(self):
        super().__init__('square_motion_node')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.motor_client = self.create_client(SetBool, '/motor_power')
        self.reset_client = self.create_client(Trigger, '/reset_odometry')
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.x = self.y = self.yaw = 0.0
        self.have_odom = False

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

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

    def wait_for_odom(self):
        while not self.have_odom:
            rclpy.spin_once(self, timeout_sec=0.1)

    def send(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.pub.publish(msg)

    def stop(self):
        self.send(0.0, 0.0)
        time.sleep(0.2)

    def drive_forward(self, distance):
        start_x, start_y = self.x, self.y
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            travelled = math.hypot(self.x - start_x, self.y - start_y)
            if distance - travelled <= POSITION_TOL:
                break
            self.send(SPEED, 0.0)
        self.stop()

    def turn(self, angle):
        target = normalize(self.yaw + angle)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            error = normalize(target - self.yaw)
            if abs(error) <= ANGLE_TOL:
                break
            self.send(0.0, math.copysign(TURN_SPEED, error))
        self.stop()

    def run(self):
        self.enable_motors()
        self.reset_odom()
        self.wait_for_odom()
        for _ in range(4):
            self.drive_forward(SIDE_LENGTH)
            self.turn(math.pi / 2)


def main():
    rclpy.init()
    node = SquareMotionNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
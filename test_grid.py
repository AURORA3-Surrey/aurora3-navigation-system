import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger

GRID_SIZE = 4       # m
CELL_SIZE = 0.3     # m
MAX_SPEED = 0.05    # m/s
MIN_SPEED = 0.018   # m/s
TURN_SPEED = 0.3    # rad/s
RAMP_DIST = 0.06    # m
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


def clamp(v, low, high):
    return max(low, min(high, v))


def smooth(t):
    t = clamp(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


class GridMotionNode(Node):
    def __init__(self):
        super().__init__('grid_motion_node')
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

    def ramped_speed(self, travelled, remaining):
        factor = min(smooth(travelled / RAMP_DIST), smooth(remaining / RAMP_DIST))
        if factor <= 0.0:
            return 0.0
        return max(MIN_SPEED, MAX_SPEED * factor)

    def drive_forward(self, distance):
        start_x, start_y, start_yaw = self.x, self.y, self.yaw
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            travelled = math.hypot(self.x - start_x, self.y - start_y)
            remaining = distance - travelled
            if remaining <= POSITION_TOL:
                break
            heading_error = normalize(start_yaw - self.yaw)
            angular = clamp(1.8 * heading_error, -0.5 * TURN_SPEED, 0.5 * TURN_SPEED)
            self.send(self.ramped_speed(travelled, remaining), angular)
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
        for row in range(GRID_SIZE):
            for _ in range(GRID_SIZE - 1):
                self.drive_forward(CELL_SIZE)
            if row == GRID_SIZE - 1:
                break
            # end of row (turn move up row and turn again)
            turn = -math.pi / 2 if row % 2 == 0 else math.pi / 2
            self.turn(turn)
            self.drive_forward(CELL_SIZE)
            self.turn(turn)


def main():
    rclpy.init()
    node = GridMotionNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
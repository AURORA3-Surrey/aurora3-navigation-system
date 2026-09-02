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
TURN_SPEED = 0.25   # rad/s
POSITION_TOL = 0.01 # m
ANGLE_TOL = 0.02    # rad
SETTLE_DELAY = 0.3  # s
CORR_GAIN = 1.5


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
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5


def velocity_factor(tau):
    # peak at 1.875
    tau = clamp(tau, 0.0, 1.0)
    return 30 * tau**2 - 60 * tau**3 + 30 * tau**4 


def duration_for_peak(distance, peak_speed):
    distance = abs(distance)
    if distance <= 0 or peak_speed <= 0:
        return 0.0
    return 1.875 * distance / peak_speed


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

    def stop(self, duration=None):
        duration = SETTLE_DELAY if duration is None else duration
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            self.send(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)

    def drive_forward(self, distance):
        T = duration_for_peak(distance, MAX_SPEED)
        if T <= 0:
            return
        start_x, start_y, start_yaw = self.x, self.y, self.yaw
        move_start = time.monotonic()
        while rclpy.ok():
            elapsed = time.monotonic() - move_start
            if elapsed >= T:
                break
            tau = elapsed / T
            v_ff = (distance / T) * velocity_factor(tau)

            # check where we are
            rclpy.spin_once(self, timeout_sec=0.0)
            travelled = math.hypot(self.x - start_x, self.y - start_y)
            expected = distance * position_factor(tau)
            v_correction = clamp(CORR_GAIN * (expected - travelled), -0.3 * MAX_SPEED, 0.3 * MAX_SPEED)

            heading_error = normalize(start_yaw - self.yaw)
            angular = clamp(1.8 * heading_error, -0.5 * TURN_SPEED, 0.5 * TURN_SPEED)

            self.send(v_ff + v_correction, angular)
            time.sleep(0.02)
        self.stop(duration=0.05)

        # correct remaining slip 
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            travelled = math.hypot(self.x - start_x, self.y - start_y)
            remaining = distance - travelled
            if remaining <= POSITION_TOL:
                break
            heading_error = normalize(start_yaw - self.yaw)
            angular = clamp(1.8 * heading_error, -0.5 * TURN_SPEED, 0.5 * TURN_SPEED)
            self.send(clamp(remaining, 0.0, MAX_SPEED), angular)
            time.sleep(0.02)
        self.stop()

    def turn(self, angle):
        T = duration_for_peak(angle, TURN_SPEED)
        if T <= 0:
            return
        start_yaw = self.yaw
        target = normalize(start_yaw + angle)
        move_start = time.monotonic()
        while rclpy.ok():
            elapsed = time.monotonic() - move_start
            if elapsed >= T:
                break
            tau = elapsed / T
            w_ff = (angle / T) * velocity_factor(tau)

            rclpy.spin_once(self, timeout_sec=0.0)
            turned = normalize(self.yaw - start_yaw)
            expected = angle * position_factor(tau)
            w_correction = clamp(CORR_GAIN * (expected - turned), -0.3 * TURN_SPEED, 0.3 * TURN_SPEED)

            self.send(0.0, w_ff + w_correction)
            time.sleep(0.02)
        self.stop(duration=0.05)

        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            error = normalize(target - self.yaw)
            if abs(error) <= ANGLE_TOL:
                break
            self.send(0.0, math.copysign(clamp(abs(error), 0.0, TURN_SPEED), error))
            time.sleep(0.02)
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
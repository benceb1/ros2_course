import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import time


class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')

        self.laser = None
        self.imu = None
        self.laser_angle = 60

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)

        self.declare_parameter('speed', 0.2)
        self.declare_parameter('omega', 1.0)

        self.pose = None
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.cb_pose,
            10)

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cb_pose(self, msg):
        self.pose = msg.pose.pose

    def laser_callback(self, msg):

        self.laser = msg

    def imu_callback(self, msg):
        self.imu = msg

    def go_straight(self, speed, distance):
        vel_msg = Twist()
        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Calculate time
        T = abs(distance/speed)     # s

            # Publish first msg and note time
        self.get_logger().info('Turtle started.')
        self.twist_pub.publish(vel_msg)
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

            # Publish msg while the calculated time is up
        while (self.get_clock().now() < when) and rclpy.ok():

            #self.get_logger().info(f'On its way...{vel_msg}')

            #if (self.laser is not None):
                #self.get_logger().info(f' laserinfo: [{str(self.laser.ranges)}]')

            if (self.laser is not None):
                angles_left = self.laser.ranges[-30:]
                angles_right = self.laser.ranges[:30]
                angles_fw = angles_left + angles_right


                turning = False
                small_distance = 0

                for a in angles_fw:
                    if a < 1:
                        small_distance = a
                        turning = True
                        break

                if (turning):
                    turning_omega = float((1 - small_distance) * 90)
                    vel_msg.angular.z = math.radians(turning_omega)
                else:
                    vel_msg.angular.z = 0.0

            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)   # loop rate

            # Set velocity to 0
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)
            #self.get_logger().info('Arrived to destination.')
        self.stop()

    def turn(self, omega, angle):
        vel_msg = Twist()

        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        if angle > 0:
            vel_msg.angular.z = math.radians(omega)
        else:
            vel_msg.angular.z = math.radians(-omega)

        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Calculate time
        T = abs(angle/omega)     # s

        # Publish first msg and note time
        #self.get_logger().info('Turtle started.')
        self.twist_pub.publish(vel_msg)
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() < when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            #self.get_logger().info('On its way...')

            if (self.laser is not None):
                self.get_logger().info(f' laserinfo: [{str(self.laser.ranges[0]>100)}]')


            rclpy.spin_once(self)   # loop rate

        # Set velocity to 0
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')
        self.stop()

    def stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    #tc.turn(90.0)
    #tc.turn(0.0)
    #tc.go_straight(1.0,4.0)

    tc.go_straight(0.3,1.0)
    #tc.turn(20.0,-90.0)

    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


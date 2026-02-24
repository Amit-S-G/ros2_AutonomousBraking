import rclpy
from rclpy.node import Node
import numpy as np
# Import standard ROS2 message types used in F1TENTH
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class AutonomousBrakingNode(Node):
    def __init__(self):
        super().__init__("aeb_node")
        self.speed = 0.0 # we assume the speed is 0 to begin
        self.ttc_threshold = 1.0 # If we go under this, start braking
        self.curr_ttc = [0.0] * 40
        self.center_distance = 0.0 # Distance of nearest object to front of car
        self.important_ranges = []
        self.angles_in_cone = []

        # We also need to init publishers and subscribers
        # We want to publish braking commands
        # And we want to subscribe to the LIDAR and spedometer
        # The format is the message protocol and the topic it's coming from, 
        # the callback function, and how often we want to callback
        self.lidar_sub = create_subscription(LaserScan, '/scan', 
                                             self.scan_callback, 10)
        self.odom_sub = create_subscription(Odometry, '/odom', 
                                             self.odom_callback, 10)
        self.brake_pub = create_publisher(AckermannDriveStamped, '/drive',
                                          self.drive_callback, 10)
        

        def scan_callback(self, msg):
            # In the LIDAR scan, we want to get the ranges 
            ranges = msg.ranges
            # This is a list of distances in a 270-degree cone around the front
            # of the car. Now, we don't want *most* of these. But we want to look
            # at the center 40 ranges because that's a reasonable mini-cone to see.
            # Additionally, consider that the ttc formula needs the angle of each
            # range
            # the angle at the middle is 0. The 20 on the right and the 20 on
            # the left need to be calc'd
            startAngle = 0-(20*msg.angle_increment)
            for i in range(0, 19):
                self.angles_in_cone[i] = startAngle
                startAngle += msg.angle_increment
            self.angles_in_cone[20] = 0
            startAngle = 0 + msg.angle_increment
            for i in range(21, 40):
                self.angles_in_cone[i] = startAngle
                startAngle += msg.angle_increment
            # this gives us the cone of angles we want
            # We also want the ranges associated with this cone
            # find the center
            middleIndex = len(ranges)/2
            # pull back 20
            startIndex = middleIndex - 20
            # get the next 40 ranges
            for i in range(0, 39):
                self.important_ranges[i] = ranges[startIndex + i]
            # okay, now we have the improtant ranges and the angles of each

        def odom_callback(self, msg):
            # just get the speed
            self.speed = msg.speed

            # based on the speed and ranges, see if we need to break
            for range, idx in self.important_ranges:
                self.curr_ttc[idx] = (range/(max(0, self.speed * np.cos(self.angles_in_cone[idx]))))


        def emergency_brake(self):
            # use the ttc formula to see if we're going to crash
            # if, so...good!!!
            # i'm just kidding. that was a test to see if you're actually
            # paying attention to the comments.
            # obviously, if we're going to crash, brake.
            # the ttc formula is given as:
            # ttc = r/max(0, -r-hat). r-hat is the range rate which is:
            # v * cos(angle of beam)
            stop_mesaage = AckermannDriveStamped()
            stop_mesaage.header.stamp = self.get_clock().now().to_msg() # get the current time
            stop_mesaage.drive.speed = 0.0 # STOP moving
            stop_mesaage.drive.acceleration = -10.0 # accelerate in the opposite direction
            self.get_logger().warn("[AEB TRIGGERED] BRAKING!", throttle_duration_sec=1.0)

    def main():
        rclpy.init(args=None)
        node = AutonomousBrakingNode()
        rclpy.spin(node)
        rclpy.shutdown()
    
    if __name__ == "__main__":
        main()

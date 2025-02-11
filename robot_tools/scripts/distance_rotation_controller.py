#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math


class DistanceAndRotationController:
    def __init__(self, target_distance, target_angle):
        rospy.init_node('distance_rotation_controller', anonymous=True)

        # Target travel distance and rotation angle
        self.target_distance = target_distance
        self.target_angle = math.radians(target_angle)  # Convert to radians
        
        # Initial position and orientation
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        
        # Current traveled distance and rotation angle
        self.current_distance = 0.0
        self.current_yaw = 0.0
        
        # Initialize velocity publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribe to odometry data
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        
        # Create velocity message
        self.vel_msg = Twist()
        
        # Control loop frequency
        self.rate = rospy.Rate(100)

    def odometry_callback(self, msg):
        """Process odometry data to compute the total traveled distance and rotation angle."""
        # Position update
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.initial_x is None or self.initial_y is None:
            # Initialize initial position
            self.initial_x = x
            self.initial_y = y
        
        self.current_distance = math.sqrt((x - self.initial_x)**2 + (y - self.initial_y)**2)

        # Orientation update
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        if self.initial_yaw is None:
            # Initialize initial orientation
            self.initial_yaw = yaw
        
        # Current rotation angle (relative to initial orientation)
        self.current_yaw = yaw - self.initial_yaw
        # Ensure the angle is within [-pi, pi] range
        self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))

    def move_to_distance(self):
        """Control the robot to move to the specified distance."""
        rospy.loginfo("Moving forward...")
        while not rospy.is_shutdown():
            if self.current_distance >= self.target_distance:
                rospy.loginfo("Target distance reached: {:.2f} meters".format(self.current_distance))
                self.vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.vel_msg)
                break
            
            # Set linear velocity
            self.vel_msg.linear.x = 0.5  # Linear velocity
            self.vel_msg.angular.z = 0.0  # Keep moving straight
            self.cmd_vel_pub.publish(self.vel_msg)
            self.rate.sleep()

    def rotate_to_angle(self):
        """Control the robot to rotate to the specified angle."""
        rospy.loginfo("Rotating...")
        while not rospy.is_shutdown():
            if abs(self.current_yaw) >= abs(self.target_angle):
                rospy.loginfo("Target angle reached: {:.2f} degrees".format(math.degrees(self.current_yaw)))
                self.vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(self.vel_msg)
                break
            
            # Set angular velocity
            self.vel_msg.linear.x = 0.2  # Stop linear velocity
            self.vel_msg.angular.z = 0.3 if self.target_angle > 0 else -0.3  # Positive or negative determines rotation direction
            self.cmd_vel_pub.publish(self.vel_msg)
            self.rate.sleep()

    def execute(self):
        """Execute movement and rotation tasks."""
        if math.fabs(self.target_distance) > 0.0:
            self.move_to_distance()
        
        if math.fabs(self.target_angle) > 0.0:
            self.rotate_to_angle()


if __name__ == "__main__":
    try:
        # Input target travel distance and rotation angle
        target_distance = float(input("Enter the target distance (in meters): "))
        target_angle = float(input("Enter the target angle (in degrees): "))
        controller = DistanceAndRotationController(target_distance, target_angle)
        rospy.loginfo("Starting robot movement...")
        controller.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")

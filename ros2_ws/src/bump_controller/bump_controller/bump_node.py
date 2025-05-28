'''
ackermann_msg = to_ackermann(self.speed, self.last_steering_angle, timestamp_unix)
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from rclpy.qos import qos_profile_sensor_data  # Quality of Service settings for real-time data
import time
from ros2_numpy import image_to_np, imu_to_np, to_ackermann
from collections import deque
import numpy as np
import time



class Bump_Controller(Node):
    def __init__(self):
        super().__init__('bump_node')
        # Define Quality of Service (QoS) for communication
        qos_profile = qos_profile_sensor_data  # Suitable for sensor data
        qos_profile.depth = 1  # Keep only the latest message

        # Create a publisher for sending AckermannDriveStamped messages to the '/autonomous/ackermann_cmd' topic
        self.publisher = self.create_publisher(AckermannDriveStamped, 'autonomous/ackermann_cmd', qos_profile)

        # Subscriber to receive camera images
        self.im_subscriber = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw',  # Topic name
            self.image_callback,  # Callback function to process incoming images
            qos_profile
        )

        # Create a subscription to listen for IMU data
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu',  # Topic name
            self.imu_callback,  # Callback function to process IMU data
            qos_profile
        )

        # Data stuff
        self.min_speed = 0.3                # const: minimum speed, used on bumps
        self.max_speed = 1.0                # const: max speed, used when not bumping
        self.speed = self.max_speed         # current speed, update while driving
        self.steer = 0.0                    # last predicted steering angle
        self.steer_gain = 1.2               # Steering effort gain
        self.bump_flag = False              # bool: do we be bumpin'?
        self.z_acc = 0                      # last z-accel measurement
        self.bump_thresh = 7.0              # z-accel threshold for bumpin'
        self.bump_count = 0                 # number of bumps detected on this bump
        self.last_bump = 0                  # UNIX timestamp of last bump



    def image_callback(self, msg):
        # Convert ROS image to numpy format
        # timestamp_unix is the image timestamp in seconds (Unix time)
        image, timestamp_unix = image_to_np(msg)

        # Run model?
        # self.steer, see_bump = self.model(image)
        self.steer, see_bump = 0.0, 0   # spoof

        # Detect new visible speedbump
        if self.bump_flag == False and see_bump:
            print("Bump spotted, prepare for bumpin'")
            self.bump_flag = True           # Set bumpin' flag
            self.speed = self.min_speed     # Reduce speed

        # Wait at least half a second between bumps
        if timestamp_unix > self.last_bump + 0.5:
            # Detect when a bump be bumped
            if self.bump_flag and self.z_acc >= self.bump_thresh:
                print("*BUMP*")
                self.bump_count = self.bump_count + 1       # Increment bump counter
                self.last_bump = timestamp_unix             # Log last bump time

        # Decide when the bump is traversed
        if self.bump_count >= 2:
            print("Bump covered, back to normal")
            self.bump_flag = False          # Lower bump flag
            self.bump_count = 0             # Reset bump counter
            self.speed = self.max_speed     # Increase speed again

        # Publish control commands
        ackermann_msg = to_ackermann(self.speed, self.steer * self.steer_gain, timestamp_unix)
        self.publisher.publish(ackermann_msg)

        return



    def imu_callback(self, msg: Imu):
        # Convert incoming IMU message to np
        data, timestamp_unix = imu_to_np(msg) # [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]

        #point, heading, timestamp_unix = pose_to_np(msg)
        print("I am IMU callback!")
        self.get_logger().info(f"x_ang: {msg.angular_velocity.x=}")

        # Log z_acceleration
        self.z_acc = data(2)

        return



def main(args=None):
    print("I am bump node!")
    rclpy.init(args=args)

    node = Bump_Controller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
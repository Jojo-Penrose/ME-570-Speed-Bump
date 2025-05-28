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
        self.curr_speed = self.min_speed    # current speed, update while driving
        self.steer = 0.0                    # last predicted steering angle
        self.bump_flag = 0                  # bool: do we be bumpin'?



    def image_callback(self, msg):

        # Convert ROS image to numpy format
        # timestamp_unix is the image timestamp in seconds (Unix time)
        image, timestamp_unix = image_to_np(msg)

        # Run model?
        # predictions = self.model(image)



    def imu_callback(self, msg: Imu):
        # Convert incoming IMU message to np
        data, timestamp_unix = imu_to_np(msg) # [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]

        #point, heading, timestamp_unix = pose_to_np(msg)
        print("I am IMU callback!")
        self.get_logger().info(f"x_ang: {msg.angular_velocity.x=}")

        # ackermann_msg = to_ackermann(self.speed, self.last_steering_angle, timestamp_unix)
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

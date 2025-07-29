#!/usr/bin/env python3
import rclpy.time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from collections import deque

def average(val):
    ans = 0
    for i in val:
      ans += i
    ans = ans/len(val)
    return ans

class MPU6050_Calibration(Node):

    def __init__(self):
        super().__init__("mpu6050_calibration")
        
        self.declare_parameter("history_size", 1000)
        self.history_size = self.get_parameter("history_size").value

        self.get_logger().info(f'history_size: {self.history_size}')

        self.count = 0
        self.percent_complete = 0 #  int((count*100)/self.history_size)

        self.gyr_x = deque(maxlen=self.history_size)
        self.gyr_y = deque(maxlen=self.history_size)
        self.gyr_z = deque(maxlen=self.history_size)

        self.acc_x = deque(maxlen=self.history_size)
        self.acc_y = deque(maxlen=self.history_size)
        self.acc_z = deque(maxlen=self.history_size)

        # ROS 2 Interface
        self.imu_sub_ = self.create_subscription(Imu, "imu/raw", self.calibration_callback, qos_profile_sensor_data)
        self.imu_acc_offset_pub_ = self.create_publisher(Vector3, "/imu/acc_offset", 10)
        self.imu_gyr_offset_pub_ = self.create_publisher(Vector3, "/imu/gyr_offset", 10)

        self.get_logger().info("mpu6050_calibration node has started ....")
    
    def calibration_callback(self, imu_msg):
        acc_offset = Vector3()
        gyr_offset = Vector3()

        self.percent_complete = int((self.count*100)/self.history_size)
        self.get_logger().info("reading imu data...  %d percent complete" % (self.percent_complete,))

        if self.count >= self.history_size:
            # gyroscope
            min_x = min(self.gyr_x)
            max_x = max(self.gyr_x)
            min_y = min(self.gyr_y)
            max_y = max(self.gyr_y)
            min_z = min(self.gyr_z)
            max_z = max(self.gyr_z)

            gx_offset = (max_x + min_x) / 2
            gy_offset = (max_y + min_y) / 2
            gz_offset = (max_z + min_z) / 2

            gyr_offset.x = gx_offset
            gyr_offset.y = gy_offset
            gyr_offset.z = gz_offset

            # accelerometer
            ax_offset = average(self.acc_x)
            ay_offset = average(self.acc_y)
            az_offset = average(self.acc_z) - 9.8

            acc_offset.x = ax_offset
            acc_offset.y = ay_offset
            acc_offset.z = az_offset

            # print
            self.get_logger().info(f'lin_acc_offset_x: {acc_offset.x}')
            self.get_logger().info(f'lin_acc_offset_y: {acc_offset.y}')
            self.get_logger().info(f'lin_acc_offset_z: {acc_offset.z}')

            self.get_logger().info(f'ang_vel_offset_x: {gyr_offset.x}')
            self.get_logger().info(f'ang_vel_offset_y: {gyr_offset.y}')
            self.get_logger().info(f'ang_vel_offset_z: {gyr_offset.z}')

            # publish offset
            self.imu_gyr_offset_pub_.publish(gyr_offset)
            self.imu_acc_offset_pub_.publish(acc_offset)

            self.get_logger().info("mpu6050_calibration node has finished succesfully")

            self.destroy_node()
            rclpy.shutdown()
        
        
        self.acc_x.append(imu_msg.linear_acceleration.x)
        self.acc_y.append(imu_msg.linear_acceleration.y)
        self.acc_z.append(imu_msg.linear_acceleration.z)

        self.gyr_x.append(imu_msg.angular_velocity.x)
        self.gyr_y.append(imu_msg.angular_velocity.y)
        self.gyr_z.append(imu_msg.angular_velocity.z)

        self.count += 1


        


def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Calibration()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
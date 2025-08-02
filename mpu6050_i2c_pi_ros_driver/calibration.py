#!/usr/bin/env python3
import rclpy.time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from collections import deque
import yaml
import os
import numpy as np

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

        self.ax_offset = 0.00
        self.ay_offset = 0.00
        self.az_offset = 0.00

        self.gx_offset = 0.00
        self.gy_offset = 0.00
        self.gz_offset = 0.00

        self.ax_variance = 0.00
        self.ay_variance = 0.00
        self.az_variance = 0.00

        self.gx_variance = 0.00
        self.gy_variance = 0.00
        self.gz_variance = 0.00

        self.calibrate = True

        self.get_logger().info(f'history_size: {self.history_size}')

        self.count = 0
        self.percent_complete = 0 #  int((count*100)/self.history_size)

        self.gyr_x = deque(maxlen=self.history_size)
        self.gyr_y = deque(maxlen=self.history_size)
        self.gyr_z = deque(maxlen=self.history_size)

        self.acc_x = deque(maxlen=self.history_size)
        self.acc_y = deque(maxlen=self.history_size)
        self.acc_z = deque(maxlen=self.history_size)

        self.gyr_var_x = deque(maxlen=self.history_size)
        self.gyr_var_y = deque(maxlen=self.history_size)
        self.gyr_var_z = deque(maxlen=self.history_size)

        self.acc_var_x = deque(maxlen=self.history_size)
        self.acc_var_y = deque(maxlen=self.history_size)
        self.acc_var_z = deque(maxlen=self.history_size)

        # ROS 2 Interface
        self.imu_raw_sub_ = self.create_subscription(Imu, "imu/raw", self.offset_calibration_callback, qos_profile_sensor_data)
        self.imu_data_sub_ = self.create_subscription(Imu, "imu/data", self.variance_compute_callback, qos_profile_sensor_data)
        self.imu_acc_offset_pub_ = self.create_publisher(Vector3, "/imu/acc_offset", 10)
        self.imu_gyr_offset_pub_ = self.create_publisher(Vector3, "/imu/gyr_offset", 10)

        self.get_logger().info("mpu6050_calibration node has started ....")
    
    def offset_calibration_callback(self, imu_msg):
        if self.calibrate:
            acc_offset = Vector3()
            gyr_offset = Vector3()

            self.percent_complete = int((self.count*100)/self.history_size)
            self.get_logger().info("calibrating offset...  %d percent complete" % (self.percent_complete,))

            if self.count >= self.history_size:
                # gyroscope
                min_x = min(self.gyr_x)
                max_x = max(self.gyr_x)
                min_y = min(self.gyr_y)
                max_y = max(self.gyr_y)
                min_z = min(self.gyr_z)
                max_z = max(self.gyr_z)

                self.gx_offset = (max_x + min_x) / 2
                self.gy_offset = (max_y + min_y) / 2
                self.gz_offset = (max_z + min_z) / 2

                gyr_offset.x = self.gx_offset
                gyr_offset.y = self.gy_offset
                gyr_offset.z = self.gz_offset

                # accelerometer
                self.ax_offset = average(self.acc_x)
                self.ay_offset = average(self.acc_y)
                self.az_offset = average(self.acc_z) - 9.8

                acc_offset.x = self.ax_offset
                acc_offset.y = self.ay_offset
                acc_offset.z = self.az_offset

                # publish offset
                self.imu_gyr_offset_pub_.publish(gyr_offset)
                self.imu_acc_offset_pub_.publish(acc_offset)

                self.get_logger().info("offset computed")

                self.calibrate = False
                self.count = 0
                self.percent_complete = 0
 
            self.acc_x.append(imu_msg.linear_acceleration.x)
            self.acc_y.append(imu_msg.linear_acceleration.y)
            self.acc_z.append(imu_msg.linear_acceleration.z)

            self.gyr_x.append(imu_msg.angular_velocity.x)
            self.gyr_y.append(imu_msg.angular_velocity.y)
            self.gyr_z.append(imu_msg.angular_velocity.z)

            self.count += 1

    def variance_compute_callback(self, imu_msg):
        if not self.calibrate:

            self.percent_complete = int((self.count*100)/self.history_size)
            self.get_logger().info("computing variance...  %d percent complete" % (self.percent_complete,))

            if self.count >= self.history_size:
                # gyroscope
                self.gx_variance = float(np.var(self.gyr_var_x))
                self.gy_variance = float(np.var(self.gyr_var_y))
                self.gz_variance = float(np.var(self.gyr_var_z))

                # accelerometer
                self.ax_variance = float(np.var(self.acc_var_x))
                self.ay_variance = float(np.var(self.acc_var_y))
                self.az_variance = float(np.var(self.acc_var_z))

                data = {
                        'mpu6050_i2c_pi_ros_driver': {
                            'ros__parameters': {
                                'frame_id': "imu",
                                'publish_frequency': 40.0,
                                'lin_acc_offset_x': self.ax_offset,
                                'lin_acc_offset_y': self.ay_offset,
                                'lin_acc_offset_z': self.az_offset,
                                'ang_vel_offset_x': self.gx_offset,
                                'ang_vel_offset_y': self.gy_offset,
                                'ang_vel_offset_z': self.gz_offset,
                                'lin_acc_variance_x': self.ax_variance,
                                'lin_acc_variance_y': self.ay_variance,
                                'lin_acc_variance_z': self.az_variance,
                                'ang_vel_variance_x': self.gx_variance,
                                'ang_vel_variance_y': self.gy_variance,
                                'ang_vel_variance_z': self.gz_variance
                            }
                        }
                    }
                
                # === Use absolute path based on current script location ===
                script_dir = os.path.dirname(os.path.abspath(__file__))
                config_dir = os.path.abspath(os.path.join(script_dir, '..', 'config'))
                os.makedirs(config_dir, exist_ok=True)

                # Full path to YAML file
                yaml_file_path = os.path.join(config_dir, 'imu_params.yaml')

                # Save to YAML
                with open(yaml_file_path, 'w') as file:
                    yaml.dump(data, file, default_flow_style=False)

                self.get_logger().info("variance computed")
                self.get_logger().info("mpu6050_calibration node has finished succesfully")

                self.calibrate = True

                self.destroy_node()
                rclpy.shutdown()
            
            
            self.acc_var_x.append(imu_msg.linear_acceleration.x)
            self.acc_var_y.append(imu_msg.linear_acceleration.y)
            self.acc_var_z.append(imu_msg.linear_acceleration.z)

            self.gyr_var_x.append(imu_msg.angular_velocity.x)
            self.gyr_var_y.append(imu_msg.angular_velocity.y)
            self.gyr_var_z.append(imu_msg.angular_velocity.z)

            self.count += 1
        


def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Calibration()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
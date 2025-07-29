#!/usr/bin/env python3
import rclpy.time
import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68


class MPU6050_Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_i2c_pi_ros_driver")

        self.declare_parameter("frame_id", "imu")
        self.declare_parameter("publish_frequency", 10.0) #Hz

        self.declare_parameter("lin_acc_offset_x", 0.00)
        self.declare_parameter("lin_acc_offset_y", 0.00)
        self.declare_parameter("lin_acc_offset_z", 0.00)

        self.declare_parameter("ang_vel_offset_x", 0.00)
        self.declare_parameter("ang_vel_offset_y", 0.00)
        self.declare_parameter("ang_vel_offset_z", 0.00)

        self.frame_id = self.get_parameter("frame_id").value
        self.publish_frequency = self.get_parameter("publish_frequency").value

        self.lin_acc_offset_x = self.get_parameter("lin_acc_offset_x").value
        self.lin_acc_offset_y = self.get_parameter("lin_acc_offset_y").value
        self.lin_acc_offset_z = self.get_parameter("lin_acc_offset_z").value

        self.ang_vel_offset_x = self.get_parameter("ang_vel_offset_x").value
        self.ang_vel_offset_y = self.get_parameter("ang_vel_offset_y").value
        self.ang_vel_offset_z = self.get_parameter("ang_vel_offset_z").value

        self.get_logger().info(f'frame_id: {self.frame_id}')
        self.get_logger().info(f'publish_frequency: {self.publish_frequency}')

        self.get_logger().info(f'lin_acc_offset_x: {self.lin_acc_offset_x}')
        self.get_logger().info(f'lin_acc_offset_y: {self.lin_acc_offset_y}')
        self.get_logger().info(f'lin_acc_offset_z: {self.lin_acc_offset_z}')

        self.get_logger().info(f'ang_vel_offset_x: {self.ang_vel_offset_x}')
        self.get_logger().info(f'ang_vel_offset_y: {self.ang_vel_offset_y}')
        self.get_logger().info(f'ang_vel_offset_z: {self.ang_vel_offset_z}')
        
        
        # I2C Interafce
        self.is_connected_ = False
        self.init_i2c()

        # ROS 2 Interface
        self.imu_raw_pub_ = self.create_publisher(Imu, "/imu/raw", qos_profile=qos_profile_sensor_data)
        self.imu_raw_msg_ = Imu()
        self.imu_raw_msg_.header.frame_id = "imu_raw"

        self.imu_pub_ = self.create_publisher(Imu, "/imu/data", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = self.frame_id
        
        self.timer_ = self.create_timer(1.0/self.publish_frequency, self.timerCallback)

        self.get_logger().info("mpu6050_i2c_pi_ros_driver node has started succesfully")

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()
            
            # Read Accelerometer raw value
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            
            # Read Gyroscope raw value
            gyr_x = self.read_raw_data(GYRO_XOUT_H)
            gyr_y = self.read_raw_data(GYRO_YOUT_H)
            gyr_z = self.read_raw_data(GYRO_ZOUT_H)
            
            # Full scale range +/- 250 degree/C as per sensitivity scale factor
            lin_acc_raw_x = acc_x / 1670.13
            lin_acc_raw_y = acc_y / 1670.13
            lin_acc_raw_z = acc_z / 1670.13

            ang_vel_raw_x = gyr_x / 7509.55
            ang_vel_raw_y = gyr_y / 7509.55
            ang_vel_raw_z = gyr_z / 7509.55

            lin_acc_x = lin_acc_raw_x - self.lin_acc_offset_x
            lin_acc_y = lin_acc_raw_y - self.lin_acc_offset_y
            lin_acc_z = lin_acc_raw_z - self.lin_acc_offset_z

            ang_vel_x = ang_vel_raw_x - self.ang_vel_offset_x
            ang_vel_y = ang_vel_raw_y - self.ang_vel_offset_y
            ang_vel_z = ang_vel_raw_z - self.ang_vel_offset_z

            self.imu_raw_msg_.linear_acceleration.x = lin_acc_raw_x
            self.imu_raw_msg_.linear_acceleration.y = lin_acc_raw_y
            self.imu_raw_msg_.linear_acceleration.z = lin_acc_raw_z
            self.imu_raw_msg_.angular_velocity.x = ang_vel_raw_x
            self.imu_raw_msg_.angular_velocity.y = ang_vel_raw_y
            self.imu_raw_msg_.angular_velocity.z = ang_vel_raw_z

            self.imu_msg_.linear_acceleration.x = lin_acc_x
            self.imu_msg_.linear_acceleration.y = lin_acc_y
            self.imu_msg_.linear_acceleration.z = lin_acc_z
            self.imu_msg_.angular_velocity.x = ang_vel_x
            self.imu_msg_.angular_velocity.y = ang_vel_y
            self.imu_msg_.angular_velocity.z = ang_vel_z

            self.imu_raw_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_raw_pub_.publish(self.imu_raw_msg_)

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
        except OSError:
            self.is_connected_ = False

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 24)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
        except OSError:
            self.is_connected_ = False
        
    def read_raw_data(self, addr):
        #Accelero and Gyro value are 16-bit
        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
        low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr+1)
        
        #concatenate higher and lower value
        value = ((high << 8) | low)
            
        #to get signed value from mpu6050
        if(value > 32768):
            value = value - 65536
        return value


def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Driver()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
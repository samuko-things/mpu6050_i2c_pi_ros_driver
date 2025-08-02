import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Process the URDF file

    imu_node = Node(
        package='mpu6050_i2c_pi_ros_driver',
        executable='mpu6050_i2c_pi_ros_driver',
        name='mpu6050_i2c_pi_ros_driver',
        output='screen',
        parameters=[
            {"publish_frequency": 40.0},
        ],
    )

    calibration_node = Node(
        package='mpu6050_i2c_pi_ros_driver',
        executable='calibration',
        name='mpu6050_calibration',
        output='screen',
        parameters=[
            {"history_size": 1000},
        ],
    )

     # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(imu_node)
    ld.add_action(calibration_node)
    
    return ld      # return (i.e send) the launch description for excecution
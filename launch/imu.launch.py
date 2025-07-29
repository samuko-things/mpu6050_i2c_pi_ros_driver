import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('mpu6050_i2c_pi_ros_driver'))
    imu_config_file = os.path.join(pkg_path,'config','imu_params.yaml')

    imu_node = Node(
        package='mpu6050_i2c_pi_ros_driver',
        executable='mpu6050_i2c_pi_ros_driver',
        name='mpu6050_i2c_pi_ros_driver',
        output='screen',
        parameters=[imu_config_file],
    )

     # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(imu_node)
    
    return ld      # return (i.e send) the launch description for excecution
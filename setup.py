from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mpu6050_i2c_pi_ros_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuko',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # 'executable_name = package_name.python_file_name:main'
            'mpu6050_i2c_pi_ros_driver = mpu6050_i2c_pi_ros_driver.mpu6050_i2c_pi_ros_driver:main',
            'calibration = mpu6050_i2c_pi_ros_driver.calibration:main'
        ],
    },
)
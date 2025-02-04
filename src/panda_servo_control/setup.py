from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'panda_servo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  
        (os.path.join('share', package_name, 'config'), glob('config/*')),    
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='efectn',
    maintainer_email='efectn@protonmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_keyboard = servo_keyboard.servo_keyboard:main',
            'joystick_servo_publisher = joystick_servo_publisher.joystick_servo_publisher:main'
        ],
    },
)

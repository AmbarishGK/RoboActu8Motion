from setuptools import find_packages, setup

package_name = 'cart_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    keywords=['ROS'],
    maintainer='Ankit Pal',
    maintainer_email='ankit.pal@sjsu.edu',
    description='A robot-agnostic teleoperation node to convert keyboard'
                'commands to Twist messages.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_twist_keyboard = cart_control.teleop_twist_keyboard:main',
            'vesc = cart_control.vesc:main'
        ],
    },
)

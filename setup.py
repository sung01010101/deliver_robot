from setuptools import setup

package_name = 'deliver_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sung',
    maintainer_email='sung@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_cmd_vel = deliver_robot.demo_cmd_vel:main',
            'esp32_scog_node = deliver_robot.esp32_scog_node:main',
            'esp32_serial_node = deliver_robot.esp32_serial_node:main',
            'get_param_n = deliver_robot.get_param_n:main',
            'get_amcl_pose = deliver_robot.get_amcl_pose:main',
            'imu_orientation = deliver_robot.imu_orientation:main',
        ],
    },
)

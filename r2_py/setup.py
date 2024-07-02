from setuptools import find_packages, setup

package_name = 'r2_py'

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
    maintainer='adyansh04',
    maintainer_email='gupta.adyansh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motor_server_fake = r2_py.motor_server_fake:main",
            "motor_client = r2_py.motor_client:main",
            "force_stop = r2_py.force_stop:main",
            "quat_to_rpy = r2_py.quaternion_to_rpy:main",
            "ps4 = r2_py.ps4:main",
            "cmd_vel_slow_pub = r2_py.cmd_vel_slow_pub:main",
            "joint_controllers = r2_py.joint_controllers:main",
            "robot_altitude_check = r2_py.robot_altitude_check:main",
            "velocity_scalar = r2_py.velocity_scalar:main",
        ],
    },
)

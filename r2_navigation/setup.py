from setuptools import find_packages, setup

package_name = 'r2_navigation'

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
    maintainer='adyansh',
    maintainer_email='gupta.adyansh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "robot_altitude_check = r2_navigation.robot_altitude_check:main",
        "navigation_server = r2_navigation.navigation_server:main",
        "rotate_and_move = r2_navigation.rotate_and_move:main",
        "scatter_balls = r2_navigation.scatter_balls:main",
        "scatter_balls_m2 = r2_navigation.scatter_mark2:main"
        ],
    },
)

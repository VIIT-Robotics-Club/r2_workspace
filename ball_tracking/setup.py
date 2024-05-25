from setuptools import find_packages, setup

package_name = 'ball_tracking'

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
        "ball_detect_sim=ball_tracking.ball_detect_sim:main",
        "ball_tracking_sim=ball_tracking.ball_tracking_sim:main",
        ],
    },
)

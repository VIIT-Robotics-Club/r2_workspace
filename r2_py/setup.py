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
            "number_publisher = r2_py.number_publisher:main",
            "number_counter = r2_py.number_counter:main",
            "motor_server_fake = r2_py.motor_server_fake:main",
            "motor_client = r2_py.motor_client:main",
        ],
    },
)

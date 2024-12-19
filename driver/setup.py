from setuptools import find_packages, setup

package_name = 'driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/driver.launch.py']),
        ('share/' + package_name + '/launch', ['launch/motors.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wodo',
    maintainer_email='moswodocanal@gmail.com',
    description='diff drive robot pkg',
    license='no_license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		    'new_motors = driver.new_motor_encoder_driver:main',

            'encoders = driver.encoders:main',
            'motors = driver.motor_driver:main',
            'regulator = driver.regulator:main',

            'cmd_vel_maker = driver.cmd_vel_teleop:main',
            'cmd_vel_encoder = driver.cmd_vel_encryptor:main',
        ],
    },
)

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
		    'motor_controller_node = driver.motor_controller:main',
            'encoder_node = driver.encoders:main',
            'diff_drive_controller_node = driver.main_controller:main',
        ],
    },
)

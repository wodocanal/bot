from setuptools import find_packages, setup

package_name = 'odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wodo',
    maintainer_email='moswodocanal@gmail.com',
    description='odometry by encoders',
    license='no license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_node = odometry.odom:main'
        ],
    },
)

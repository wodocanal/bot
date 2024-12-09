from setuptools import find_packages, setup

package_name = 'navigation'

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
    description='try to make a navigation for my robot',
    license='no license',
    tests_require=['pytest'],
    entry_points={'console_scripts': []
    },
)

from setuptools import find_packages, setup

package_name = 'usbcan_node'

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
    maintainer='rrn',
    maintainer_email='wenhuihui20020822@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usbcan_node = usbcan_node.usbcan_node:main',
            'car_control_node = usbcan_node.car_control_node:main',
            'car_control_gui_node = usbcan_node.car_control_gui_node:main',
        ],
    },
)

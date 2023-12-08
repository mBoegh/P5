import os
from glob import glob
from setuptools import setup

package_name = 'EXONET'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include launch file
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boegh',
    maintainer_email='mboghl21@student.aau.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'server = EXONET.NODE_server:main',
                'gui = EXONET.NODE_gui:main',
                'serial_communicator = EXONET.NODE_serial_communicator:main',
                'controller = EXONET.NODE_controller:main',
                'angle_control = EXONET.NODE_position_control:main',
                'sercon = EXONET.serial_connect:main',
                'arrow_control = EXONET.desired_position:main'
                
        ],

    },
)

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
                'settings = EXONET.NODE_settings:main',
                'server = EXONET.NODE_server:main',
                'visualizer = EXONET.NODE_visualizer:main',
                'serial_communication = EXONET.NODE_serial_communication:main',
                'controller = EXONET.NODE_controller:main'
        ],

    },
)

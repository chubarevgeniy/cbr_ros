from setuptools import find_packages, setup

package_name = 'cbr'

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
    maintainer='evgenii',
    maintainer_email='jekaven2014@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cli_controller = cbr.cli_controller:main',
            'cbr_odrive_can_bridge = cbr.cbr_odrive_can_bridge:main',
            'can_bridge = cbr.can_bridge:main',
            'debug_joy_controller = cbr.debug_joy_controller:main'
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'chess_manager'

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
    maintainer='angel',
    maintainer_email='angelengineer@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'node = chess_manager.node:main',
            'robot_move_publisher = chess_manager.robot_move_publisher:main',
            'robot_move = chess_manager.robot_move:main',
        ],
    },
)

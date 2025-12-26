from setuptools import setup
from glob import glob
import os

package_name = 'kat_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ✅ config 폴더 전체를 share에 설치
        (os.path.join('share', package_name, 'config', 'open_manipulator_x'),
         glob('config/open_manipulator_x/*.yaml')),

        # ✅ ros2_control 폴더(xacro 등)도 share에 설치 (이게 핵심)
        (os.path.join('share', package_name, 'ros2_control'),
         glob('ros2_control/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ydawn',
    maintainer_email='ydawn@example.com',
    description='kat_control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'computer_move_listener = kat_control.computer_move_listener:main',
        ],
    },
)

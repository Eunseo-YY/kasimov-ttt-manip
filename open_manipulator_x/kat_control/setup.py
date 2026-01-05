from setuptools import setup
from glob import glob
import os

package_name = 'kat_control'

def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ✅ launch 파일 설치 (이거 없으면 ros2 launch가 런치를 못 찾음)
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        # ✅ config 설치
        (os.path.join('share', package_name, 'config', 'open_manipulator_x'),
         glob('config/open_manipulator_x/*.yaml')),

        # ✅ ros2_control 전체 재귀 설치 (하위폴더 포함)
        (os.path.join('share', package_name, 'ros2_control'),
         package_files('ros2_control')),
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

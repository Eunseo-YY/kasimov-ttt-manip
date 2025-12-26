from setuptools import setup

package_name = 'kat_tictactoe'

setup(
    name=package_name,
    version='0.1.0',
    # find_packages 대신 직접 이름을 적어주는 것이 오류가 적습니다.
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 경로 생성을 위해 os, glob import가 상단에 있어야 합니다.
        ('share/' + package_name + '/launch', ['launch/tictactoe.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ydawn',
    maintainer_email='ydawn@example.com',
    description='kat tictactoe package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'game_manager = kat_tictactoe.game_manager:main',
            'player_interface = kat_tictactoe.player_interface:main',
            'tictactoe_logic = kat_tictactoe.tictactoe_logic:main',
        ],
    },
)
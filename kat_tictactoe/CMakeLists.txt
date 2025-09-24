cmake_minimum_required(VERSION 3.8)
project(kat_tictactoe)

# 경고 옵션(선택)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcply REQUIRED)

# 파이썬 패키지(kat_tictactoe 디렉터리) 설치
ament_python_install_package(${PROJECT_NAME})

# 실행 스크립트 설치 (ros2 run/launch 에서 executable로 사용)
install(PROGRAMS
  kat_tictactoe/game_manager.py
  kat_tictactoe/player_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

# launch/ 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()


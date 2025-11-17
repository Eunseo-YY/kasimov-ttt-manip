cmake_minimum_required(VERSION 3.8)
project(kat_tictactoe)

# 선택: C++ 안 써도 있어도 상관 없음
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python 패키지 설치 (src/kat_tictactoe 를 패키지로 인식)
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# ros2 run kat_tictactoe game_manager 이런 식으로 실행할 스크립트 설치
install(PROGRAMS
  src/${PROJECT_NAME}/game_manager.py
  src/${PROJECT_NAME}/player_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


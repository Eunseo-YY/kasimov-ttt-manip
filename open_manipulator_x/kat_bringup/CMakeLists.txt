cmake_minimum_required(VERSION 3.8)
project(kat_bringup)

find_package(ament_cmake REQUIRED)

# (선택) C++ 노드 있으면 유지
find_package(rclcpp REQUIRED)

# ✅ launch/config/world 같은 리소스 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

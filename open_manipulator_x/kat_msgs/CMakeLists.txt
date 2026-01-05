cmake_minimum_required(VERSION 3.8)
project(kat_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 메시지/서비스 정의 - 사용 중인 모든 .msg 파일을 여기에 추가해야 합니다.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoardState.msg"
  "msg/GameStart.msg"
  "msg/ComputerMove.msg"
  "msg/GameResult.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
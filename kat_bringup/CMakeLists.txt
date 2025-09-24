cmake_minimum_required(VERSION 3.8)
project(kat_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

install(
  DIRECTORY launch config omx_bringup
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY worlds/
        DESTINATION share/${PROJECT_NAME}/worlds)
ament_package()

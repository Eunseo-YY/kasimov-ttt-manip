cmake_minimum_required(VERSION 3.8)
project(kat_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 리소스 설치: urdf/ meshes/ rviz/ 있으면 함께 설치 권장

install(DIRECTORY urdf meshes 
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()


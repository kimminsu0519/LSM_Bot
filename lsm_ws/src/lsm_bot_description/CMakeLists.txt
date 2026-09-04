cmake_minimum_required(VERSION 3.5)
project(lsm_bot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds urdf config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

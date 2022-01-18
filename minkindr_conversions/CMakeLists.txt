cmake_minimum_required(VERSION 3.5)
project(minkindr_conversions)

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include)

ament_package()

install(
  DIRECTORY include/
  DESTINATION include
)
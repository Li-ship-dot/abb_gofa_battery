cmake_minimum_required(VERSION 3.22)
project(abb_gofa_support)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

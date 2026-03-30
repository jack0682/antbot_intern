# Copyright 2026 ROBOTIS AI CO., LTD.
# Licensed under the Apache License, Version 2.0
cmake_minimum_required(VERSION 3.5)
project(antbot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

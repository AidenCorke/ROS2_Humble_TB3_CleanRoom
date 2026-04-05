cmake_minimum_required(VERSION 3.8)
project(evaluator_core)

find_package(ament_cmake REQUIRED)

install(PROGRAMS bin/evaluator_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.10)
project(ros-curses)

# general build options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find curses
find_package(Curses REQUIRED COMPONENTS panels)

# core ros-curses library
add_library(${PROJECT_NAME} src/ros-curses.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} ${CURSES_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Werror)

# include tests
add_subdirectory(tests)
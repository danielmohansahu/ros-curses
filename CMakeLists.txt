cmake_minimum_required(VERSION 3.10)
project(ros-curses)

# general build options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find curses
find_package(Curses REQUIRED COMPONENTS)
find_library(CURSES_PANELS_LIBRARIES libpanel.so libpanelw.so HINTS ${NCURSES_LIBDIR} REQUIRED)

# core ros-curses library
set(SRC_FILES
  src/panels/panel_base.cpp
  src/panels/test_panel.cpp
  src/computational_graph.cpp
  src/display.cpp
  src/ros_curses.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} ${CURSES_LIBRARIES} ${CURSES_PANELS_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Werror -Wpedantic)

# demonstration executable for GUI testing
set(DEMO_FILES
  src/demo/demo_gui.cpp
)
add_executable(demo ${DEMO_FILES})
target_link_libraries(demo ${PROJECT_NAME})

# include tests
add_subdirectory(tests)
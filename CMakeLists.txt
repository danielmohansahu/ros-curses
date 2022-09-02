cmake_minimum_required(VERSION 3.10)
project(ros_curses)

# general build options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find curses
find_package(Curses REQUIRED COMPONENTS)
find_library(CURSES_PANELS_LIBRARIES libpanel.so libpanelw.so HINTS ${NCURSES_LIBDIR} REQUIRED)

# core ros-curses GUI library
set(SRC_FILES
  src/panels/header_panel.cpp
  src/panels/help_panel.cpp
  src/panels/initialization_panel.cpp
  src/panels/node_info_panel.cpp
  src/panels/node_list_panel.cpp
  src/panels/panel_base.cpp
  src/panels/service_info_panel.cpp
  src/panels/service_list_panel.cpp
  src/panels/test_panel.cpp
  src/panels/topic_info_panel.cpp
  src/panels/topic_list_panel.cpp
  src/computational_graph.cpp
  src/display.cpp
  src/ros_curses.cpp
)

add_library(${PROJECT_NAME}_gui STATIC ${SRC_FILES})
target_include_directories(${PROJECT_NAME}_gui PUBLIC include)
target_link_libraries(${PROJECT_NAME}_gui ${CURSES_LIBRARIES} ${CURSES_PANELS_LIBRARIES})
target_compile_options(${PROJECT_NAME}_gui PRIVATE -Wall -Werror -Wpedantic)

# demonstration executable for GUI testing
set(DEMO_FILES
  src/demo/demo_gui.cpp
)
add_executable(demo ${DEMO_FILES})
target_link_libraries(demo ${PROJECT_NAME}_gui)

# core ros-curses executable
set(MAIN_FILES
  src/parsers/ros1.cpp
  src/ros_curses.cpp
)
add_executable(ros-curses ${MAIN_FILES})
target_link_libraries(ros-curses ${PROJECT_NAME}_gui)

# include tests
add_subdirectory(tests)
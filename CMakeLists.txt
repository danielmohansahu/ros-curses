cmake_minimum_required(VERSION 3.10)
project(ros_curses)

# general build options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find curses
find_package(Curses REQUIRED COMPONENTS)
find_library(CURSES_PANELS_LIBRARIES libpanel.so libpanelw.so HINTS ${NCURSES_LIBDIR} REQUIRED)

# XMLRPC for ROS1
find_package(xmlrpcpp REQUIRED)

# YAMLCPP for saving / loading
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

# core ros-curses GUI library
set(SRC_FILES
  src/panels/base/panel_base.cpp
  src/panels/header_panel.cpp
  src/panels/help_panel.cpp
  src/panels/initialization_panel.cpp
  src/panels/node_info_panel.cpp
  src/panels/param_info_panel.cpp
  src/panels/service_info_panel.cpp
  src/panels/topic_info_panel.cpp
  src/computational_graph.cpp
  src/display.cpp
)

add_library(${PROJECT_NAME}_gui STATIC ${SRC_FILES})
target_include_directories(${PROJECT_NAME}_gui PUBLIC include)
target_link_libraries(${PROJECT_NAME}_gui ${CURSES_LIBRARIES} ${CURSES_PANELS_LIBRARIES} ${YAMLCPP_LIBRARIES})
target_compile_options(${PROJECT_NAME}_gui PRIVATE -Wall -Werror -Wpedantic -fconcepts)

# demonstration executable for GUI testing
set(DEMO_FILES
  src/executables/demo_gui.cpp
)
add_executable(demo ${DEMO_FILES})
target_link_libraries(demo ${PROJECT_NAME}_gui)

# core ros-curses executable
set(ROS1_FILES
  src/parsers/ros1.cpp
  src/parsers/xml_client_wrapper.cpp
)
add_executable(ros-curses ${ROS1_FILES} src/executables/ros_curses.cpp)
target_link_libraries(ros-curses ${PROJECT_NAME}_gui ${xmlrpcpp_LIBRARIES})
target_compile_options(ros-curses PRIVATE -Wall -Werror -Wpedantic)

# utility executables for saving state
add_executable(save-state ${ROS1_FILES} src/executables/save_state.cpp)
target_link_libraries(save-state ${PROJECT_NAME}_gui ${xmlrpcpp_LIBRARIES} ${YAMLCPP_LIBRARIES})
target_compile_options(save-state PRIVATE -Wall -Werror -Wpedantic)

# utility executables for loading a saved state
add_executable(load-state ${ROS1_FILES} src/executables/load_state.cpp)
target_link_libraries(load-state ${PROJECT_NAME}_gui ${xmlrpcpp_LIBRARIES} ${YAMLCPP_LIBRARIES})
target_compile_options(load-state PRIVATE -Wall -Werror -Wpedantic)

# install executables
install(TARGETS ros-curses save-state load-state DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# include tests
# add_subdirectory(tests)

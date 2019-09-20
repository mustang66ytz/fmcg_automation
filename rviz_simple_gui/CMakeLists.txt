cmake_minimum_required(VERSION 2.8.3)
project(rviz_simple_gui)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rviz 
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  std_msgs
  geometry_msgs
  message_generation
  rviz_visual_tools
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MoveitPlanner.srv
  AddObstacle.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package( 
  CATKIN_DEPENDS roscpp rviz message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

set(rviz_simple_gui_SRCS 
 src/simple_panel.cpp
 src/simple_widget.cpp
)

set(rviz_simple_gui_HDRS
  include/rviz_simple_gui/simple_panel.h
  include/rviz_simple_gui/simple_widget.h 
)

set(rviz_simple_gui_UIS
 src/uis/simple_widget.ui 
)

set(rviz_simple_gui_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_definitions(-DQT_NO_KEYWORDS)
qt5_wrap_cpp(rviz_simple_gui_MOCS ${rviz_simple_gui_HDRS})
qt5_wrap_ui(rviz_simple_gui_UIS_H ${rviz_simple_gui_UIS})

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${rviz_simple_gui_INCLUDE_DIRECTORIES}
  /usr/local/include
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${rviz_simple_gui_UIS_H}
  ${rviz_simple_gui_MOCS}
  ${rviz_simple_gui_SRCS}
)
# add library for moveit motion planning
add_library(
  motion_planning
  include/rviz_simple_gui/moveit_planner.h
  src/moveit_planner.cpp
)
# add library for planning scene construction
add_library(
  scene_construction
  include/rviz_simple_gui/planning_scene.h
  src/planning_scene.cpp
)
add_executable(${PROJECT_NAME}_moveit_node src/moveit_node.cpp)
## add motion planning service server
add_executable(${PROJECT_NAME}_plan_server_node src/motion_plan_server.cpp)
## add collision obstacle creator service server
add_executable(${PROJECT_NAME}_obstacle_server_node src/add_obstacle_server.cpp)
## add collision scene constructor
add_executable(${PROJECT_NAME}_add_obstacle_node src/collision_publisher.cpp)
## add end-effector position tracker
add_executable(${PROJECT_NAME}_position_tracking_node src/position_tracker.cpp)

target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES}
                      Qt5::Widgets
                      Qt5::Gui 
                      scene_construction
                      )
target_link_libraries(${PROJECT_NAME}_moveit_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_plan_server_node motion_planning ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_obstacle_server_node scene_construction ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_add_obstacle_node scene_construction ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_position_tracking_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})


find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

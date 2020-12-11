cmake_minimum_required(VERSION 3.0.2)
project(social_move_base)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	actionlib
	std_msgs
	geometry_msgs
  actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

## Generate actions in the 'action' folder
add_action_files(
  FILES
	SocialMoveBase.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
 	std_msgs
	geometry_msgs
	actionlib_msgs
)

catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES social_move_base
 CATKIN_DEPENDS actionlib_msgs roscpp std_msgs geometry_msgs
 # DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(social_move_base_node src/social_move_base_node.cpp)
target_link_libraries(social_move_base_node ${catkin_LIBRARIES})
add_dependencies(social_move_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

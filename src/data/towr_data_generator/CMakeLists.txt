cmake_minimum_required(VERSION 3.0.2)
project(towr_data_generator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2_msgs
  std_msgs
  xpp_msgs
  message_generation
  tf
  tf2
)

## Generate messages in the 'msg' folder
add_message_files(FILES towrGen_msgs.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS xpp_msgs roscpp
#  DEPENDS system_lib
)
###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(TOWR_data_gen src/TOWR_data_generator.cpp)
add_executable(TOWR_terrain_data src/TOWR_terrain.cpp)
# add_dependencies(TOWR_data_gen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(TOWR_data_gen
  ${catkin_LIBRARIES}
)
target_link_libraries(TOWR_terrain_data
  ${catkin_LIBRARIES}
)

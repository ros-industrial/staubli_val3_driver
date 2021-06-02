cmake_minimum_required(VERSION 2.8.3)
project(staubli_val3_driver)

# enable C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# check build type
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: No build type specified. Selecting 'Release' for best performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

# required definitions for simple_message
add_definitions(-DROS=1)
add_definitions(-DLINUXSOCKETS=1)

# find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  industrial_msgs
  industrial_robot_client
  roscpp
  sensor_msgs
  simple_message
  staubli_msgs
)

# export catkin dependencies
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    control_msgs
    industrial_msgs
    industrial_robot_client
    message_runtime
    roscpp
    sensor_msgs
    simple_message
    staubli_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


###########
## Build ##
###########

# Robot state node
add_executable(staubli_robot_state
    src/robot_state_node.cpp
    src/industrial_robot_client/joint_feedback_relay_handler.cpp)
target_link_libraries(staubli_robot_state
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(staubli_robot_state
  PROPERTIES OUTPUT_NAME robot_state
  PREFIX "")

# System interface node
add_executable(staubli_system_interface
    src/system_interface_node.cpp
    src/system_interface.cpp
    src/simple_message/set_drive_power_message.cpp)
target_link_libraries(staubli_system_interface
    simple_message
    ${catkin_LIBRARIES})
set_target_properties(staubli_system_interface
  PROPERTIES OUTPUT_NAME system_interface
  PREFIX "")

# I/O interface node
add_executable(staubli_io_interface
    src/io_interface_node.cpp
    src/io_interface.cpp
    src/simple_message/io_states.cpp
    src/simple_message/read_io_message.cpp
    src/simple_message/write_single_io.cpp
    src/simple_message/write_single_io_message.cpp)
add_dependencies(staubli_io_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(staubli_io_interface
    simple_message
    ${catkin_LIBRARIES})
set_target_properties(staubli_io_interface
  PROPERTIES OUTPUT_NAME io_interface
  PREFIX "")


#############
## Install ##
#############

install(TARGETS
  staubli_robot_state
  staubli_system_interface
  staubli_io_interface
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch val3
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test.xml)
endif()

cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  simulator
  nav_msgs
)

catkin_package(CATKIN_DEPENDS roscpp std_msgs simulator nav_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/osqp
)

add_executable(mpc_node src/mpc_node.cpp)

target_link_libraries(mpc_node
  ${catkin_LIBRARIES}
  osqp
)



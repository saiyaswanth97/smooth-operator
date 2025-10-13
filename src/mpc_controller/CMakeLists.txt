cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  simulator
  nav_msgs
)

find_package(OsqpEigen REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(CATKIN_DEPENDS roscpp std_msgs simulator nav_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/local/include/osqp
  /usr/include/eigen3
)

add_executable(mpc_node src/mpc_node.cpp)
# add_executable(mpc_node src/mpc_node.cpp src/mpc_solver_new1.cpp)

target_link_libraries(mpc_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  OsqpEigen::OsqpEigen
)

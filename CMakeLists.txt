cmake_minimum_required(VERSION 3.0.2)
project(my_ct_project)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ct_core
  # ct_models
  ct_optcon
  # ct_rbd
  roscpp
  roslib
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES my_ct_project
 CATKIN_DEPENDS ct_core roscpp roslib
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  generated
  ${catkin_INCLUDE_DIRS}
)

add_executable(masspoint src/IntMasspoint.cpp)
target_link_libraries(masspoint ct_core)

add_executable(cnt_masspoint src/IntCntMasspoint.cpp)
target_link_libraries(cnt_masspoint ct_core)

add_executable(cartpole src/CartpoleDynamicsLinearization.cpp)
target_link_libraries(cartpole ct_core ${catkin_LIBRARIES})

add_executable(cnt_cartpole src/IntCntCartpole.cpp generated/CartpoleSystemLinearized.cpp)
target_link_libraries(cnt_cartpole ct_optcon ${catkin_LIBRARIES})
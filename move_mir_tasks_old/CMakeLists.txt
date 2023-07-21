cmake_minimum_required(VERSION 3.0.2)
project(move_mir_tasks)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(solution_01 src/solution_01.cpp)
target_link_libraries(solution_01 ${catkin_LIBRARIES})

add_executable(solution_02 src/solution_02.cpp)
target_link_libraries(solution_02 ${catkin_LIBRARIES})

add_executable(solution_03 src/solution_03.cpp)
target_link_libraries(solution_03 ${catkin_LIBRARIES})

add_executable(solution_04 src/solution_04.cpp)
target_link_libraries(solution_04 ${catkin_LIBRARIES})
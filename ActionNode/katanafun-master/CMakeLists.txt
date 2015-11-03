cmake_minimum_required(VERSION 2.8.3)
project(katanafun)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ymca src/ymca.cpp)
target_link_libraries(ymca ${catkin_LIBRARIES})
add_dependencies(ymca ${catkin_EXPORTED_TARGETS})

install(TARGETS ymca 
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

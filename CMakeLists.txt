cmake_minimum_required(VERSION 3.0.2)

set(PROJECT_NAME goal_seeker)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_DEPENDENCIES 
  roscpp  
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  tf2
  icuas23_competition
  path_planner
  # backward_ros
)

find_package(catkin REQUIRED COMPONENTS
  ${PROJECT_DEPENDENCIES}
)

find_package(Eigen3)

set(SOURCE_FILES
  src/goal_seeker.cpp
)

catkin_package(
  INCLUDE_DIRS include include/${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
)

include_directories( 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include
  include/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_node src/goal_seeker_node.cpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h")

# install(DIRECTORY migration_rules
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# add_custom_target(${PROJECT_NAME}
#             SOURCES ${DRONEMSGS_HEADER_FILES}
#             )

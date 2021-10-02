cmake_minimum_required(VERSION 3.0.2)
project(makeResult)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  pcl_ros
  nav_msgs
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)


catkin_package(
  DEPENDS PCL
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

add_executable(makeResult src/makeResult.cpp)
add_dependencies(makeResult ${catkin_EXPORTED_TARGETS})
target_link_libraries(makeResult ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)
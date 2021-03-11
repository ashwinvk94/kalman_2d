cmake_minimum_required(VERSION 2.8.3)
project(kalman_2d)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
# find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kalman_2d
  CATKIN_DEPENDS
    roscpp
    std_msgs    
  # DEPENDS
  #   Eigen3
)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
   src/kalman.cpp
)

add_executable(kalman src/kalman.cpp)
add_executable(measurement_publisher src/measurement_publisher.cpp)

target_link_libraries(kalman
   ${PROJECT_NAME}
)
target_link_libraries(measurement_publisher
   ${PROJECT_NAME}
)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
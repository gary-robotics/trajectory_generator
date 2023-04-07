cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

SET(CMAKE_BUILD_TYPE Debug)  
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

# Eigen
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  message(STATUS ${EIGEN3_INCLUDE_DIRS})
endif()

find_package(Ceres REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# OpenCV
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message(STATUS ${OpenCV_INCLUDE_DIRS})
endif()
include_directories(${OpenCV_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}
#   src/initial_guess.cpp
#   src/main.cpp
#   src/motion_model.cpp
#   src/trajectory_generator.cpp
# )

add_executable(ceres_optimization
  src/initial_guess.cpp
  src/motion_model.cpp
  src/ceres_optimization.cpp
)
target_link_libraries(ceres_optimization Ceres::ceres)

add_executable(trajectory_optimization
  src/costmap_lite.cpp
  src/debug_gradient.cpp
)
target_link_libraries(trajectory_optimization Ceres::ceres ${OpenCV_LIBS})
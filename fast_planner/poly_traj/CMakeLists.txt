cmake_minimum_required(VERSION 3.5)
project(poly_traj)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(DEPENDENCIES
  eigen3_cmake_module
  rclcpp
  std_msgs
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_library( poly_traj SHARED
    src/polynomial_traj.cpp 
    )
ament_target_dependencies(poly_traj ${DEPENDENCIES})

install(TARGETS poly_traj
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(${DEPENDENCIES})
ament_export_libraries(poly_traj)
ament_export_include_directories(include)
ament_package()
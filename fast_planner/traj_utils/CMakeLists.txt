cmake_minimum_required(VERSION 3.5)
project(traj_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bspline REQUIRED)
find_package(path_searching REQUIRED)
find_package(poly_traj REQUIRED)
find_package(plan_env REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

set(DEPENDENCIES
    rclcpp
    bspline
    path_searching
    poly_traj
    plan_env
    visualization_msgs
    eigen3_cmake_module
)

include_directories( 
    include 
    ${Eigen3_INCLUDE_DIRS} 
)

add_library( traj_utils
    src/planning_visualization.cpp)
ament_target_dependencies(traj_utils ${DEPENDENCIES})

install(TARGETS traj_utils
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
ament_export_libraries(traj_utils)
ament_export_include_directories(include)
ament_package()
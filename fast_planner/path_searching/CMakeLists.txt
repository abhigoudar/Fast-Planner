cmake_minimum_required(VERSION 3.5)
project(path_searching)
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
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plan_env REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(PCL REQUIRED)

set(DEPENDENCIES
    rclcpp
    std_msgs
    visualization_msgs
    cv_bridge
    eigen3_cmake_module
    plan_env
)

include_directories( 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( path_searching 
    src/kinodynamic_astar.cpp 
    src/astar.cpp
    src/topo_prm.cpp
    )
ament_target_dependencies(path_searching ${DEPENDENCIES})

install(TARGETS path_searching
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(${DEPENDENCIES})
ament_export_libraries(path_searching)
ament_export_include_directories(include)
ament_package()
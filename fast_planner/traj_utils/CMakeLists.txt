cmake_minimum_required(VERSION 3.5)
project(traj_utils)

add_compile_options("-std=c++14")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(bspline REQUIRED)
find_package(path_searching REQUIRED)
find_package(poly_traj REQUIRED)
find_package(plan_env REQUIRED)

set(DEPENDENCIES
  bspline
  path_searching
  poly_traj
  plan_env
)

include_directories( 
    include 
    ${Eigen3_INCLUDE_DIRS} 
)

add_library( traj_utils SHARED
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

ament_export_dependencies(${DEPENDENCIES})
ament_export_libraries(traj_utils)
ament_export_include_directories(include)
ament_package()
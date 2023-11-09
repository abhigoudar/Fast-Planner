cmake_minimum_required(VERSION 3.5)
project(path_searching)

add_compile_options("-std=c++14")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(plan_env REQUIRED)

find_package(PCL REQUIRED)

set(DEPENDENCIES
    plan_env)

include_directories( 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( path_searching SHARED
    src/kinodynamic_astar.cpp 
    src/astar.cpp
    src/topo_prm.cpp)
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
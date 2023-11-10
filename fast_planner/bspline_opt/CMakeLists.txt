cmake_minimum_required(VERSION 3.5)
project(bspline_opt)

add_compile_options("-std=c++14")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(plan_env REQUIRED)

find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

set(
    DEPENDENCIES
    plan_env
)

include_directories( 
    SYSTEM 
    include 
    ${Eigen3_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIR}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library(bspline_opt SHARED
    src/bspline_optimizer.cpp
    )
target_link_libraries(bspline_opt ${NLOPT_LIBRARIES})
ament_target_dependencies(bspline_opt ${DEPENDENCIES})

install(TARGETS bspline_opt
    LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(${DEPENDENCIES})
ament_export_libraries(bspline_opt)
ament_export_include_directories(include)
ament_package()

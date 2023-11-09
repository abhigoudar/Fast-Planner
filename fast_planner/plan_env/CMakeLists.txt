cmake_minimum_required(VERSION 3.5)
project(plan_env)


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

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(PCL REQUIRED)

set(
    DEPENDENCIES
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    cv_bridge
    message_filters
    PCL
)

include_directories( 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env 
    src/sdf_map.cpp 
    src/obj_predictor.cpp 
    src/edt_environment.cpp 
    src/raycast.cpp
    )
ament_target_dependencies(plan_env ${DEPENDENCIES})
target_link_libraries( plan_env
    ${PCL_LIBRARIES})  

add_executable(obj_generator
    src/obj_generator.cpp 
    )
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES})
ament_target_dependencies(obj_generator ${DEPENDENCIES})

install(TARGETS plan_env
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS obj_generator
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
ament_export_libraries(plan_env)
ament_export_include_directories(include)
ament_package()
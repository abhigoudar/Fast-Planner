cmake_minimum_required(VERSION 3.5)
project(plan_manage)

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
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(poly_traj)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

set(DEPENDENCIES
  rclcpp
  path_searching
  bspline
  bspline_opt
  traj_utils
  poly_traj
  cv_bridge
  visualization_msgs
)
# Generate messages in the 'msg' folder
set(msg_files
  "msg/Bspline.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces geometry_msgs
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(fast_planner_node
  src/fast_planner_node.cpp 
  src/kino_replan_fsm.cpp
  src/topo_replan_fsm.cpp
  src/planner_manager.cpp
)

ament_target_dependencies(fast_planner_node ${DEPENDENCIES})
target_link_libraries(fast_planner_node ${NLOPT_LIBRARIES})
rosidl_target_interfaces(fast_planner_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add_executable(traj_server src/traj_server.cpp)
# target_link_libraries(traj_server ${catkin_LIBRARIES})
# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS fast_planner_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# install(TARGETS traj_server
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(${DEPENDENCIES})
ament_export_libraries(plan_manage)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
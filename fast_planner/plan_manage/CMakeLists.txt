cmake_minimum_required(VERSION 3.5)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(ament_cmake REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(poly_traj)
find_package(rosidl_default_generators REQUIRED)

find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

set(DEPENDENCIES
  path_searching
  bspline
  bspline_opt
  traj_utils
  poly_traj
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
   RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS traj_server
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
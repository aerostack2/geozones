cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME geofencing)
project(${PROJECT_NAME} VERSION 0.2.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  as2_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  geographic_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
  include/geofence.hpp
  include/json.hpp
)

set(SOURCE_CPP_FILES
  src/geofencing.cpp
)

# set(INCLUDE_HPP_FILES
# include/${PROJECT_NAME}/as2_node_template_libs.hpp
# )
add_executable(${PROJECT_NAME}_node src/geofencing_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

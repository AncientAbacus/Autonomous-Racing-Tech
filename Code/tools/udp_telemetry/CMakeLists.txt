cmake_minimum_required(VERSION 3.5)
project(udp_telemetry)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(deep_orange_msgs REQUIRED)
# find_package(tum_msgs REQUIRED)

set(${PROJECT_NAME}_SRC
  src/telemetry.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRC}
)

set(${PROJECT_NAME}_NODE_PLUGIN_CLASS "race::telemetry::UdpTelemetry")
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ${${PROJECT_NAME}_NODE_PLUGIN_CLASS}
  EXECUTABLE ${PROJECT_NAME}_node_exe
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)

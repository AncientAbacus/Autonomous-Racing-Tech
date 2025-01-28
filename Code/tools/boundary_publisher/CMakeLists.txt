cmake_minimum_required(VERSION 3.5)
project(boundary_publisher)

# Default to C99.
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

# Require that dependencies from package.xml be available.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/boundary_publisher_node.cpp
)

# Create this project as a library.
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRC}
)

# Register this project as a component node.
set(${PROJECT_NAME}_NODE_PLUGIN_CLASS "race::tools::boundary_publisher::BoundaryPublisherNode")
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

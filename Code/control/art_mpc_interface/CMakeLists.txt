cmake_minimum_required(VERSION 3.8)
project(art_mpc_interface)

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require that dependencies from package.xml be available.
find_package(casadi REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/ros_param_loader.cpp
  src/art_mpc_interface_node.cpp
)

set(${PROJECT_NAME}_HEADER
  include/art_mpc_interface/art_mpc_interface_config.hpp
  include/art_mpc_interface/ros_param_loader.hpp
  include/art_mpc_interface/art_mpc_interface_node.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRC}
  ${${PROJECT_NAME}_HEADER}
)

target_link_libraries(${PROJECT_NAME} casadi)

ament_auto_add_executable(${PROJECT_NAME}_node_exe
  ${${PROJECT_NAME}_SRC}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(TEST_SOURCES test/test_art_mpc_interface.cpp)
  set(TEST_INTERFACE_EXE test_art_mpc_interface)
  ament_add_gtest(${TEST_INTERFACE_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_INTERFACE_EXE} ${PROJECT_NAME})
endif()

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  param
  launch
)

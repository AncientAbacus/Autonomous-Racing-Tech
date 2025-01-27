cmake_minimum_required(VERSION 3.8)
project(art_autoware_interface)

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

ament_auto_add_library(${PROJECT_NAME}_node SHARED
src/art_autoware_interface.cpp
)

# Create this project as an executable.
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "race::ArtAutowareInterfaceNode"
  EXECUTABLE ${PROJECT_NAME}
  EXECUTOR SingleThreadedExecutor
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   set(TEST_SOURCES test/test_rvc.cpp)
#   set(TEST_RVC_EXE test_rvc)
#   ament_add_gtest(${TEST_RVC_EXE} ${TEST_SOURCES})
#   target_link_libraries(${TEST_RVC_EXE} ${PROJECT_NAME})
# endif()

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)

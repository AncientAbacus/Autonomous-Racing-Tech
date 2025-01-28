cmake_minimum_required(VERSION 3.8)
project(velocity_estimation)

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/velocity_estimation.cpp
  src/velocity_estimation_node.cpp
)

# Create this project as an executable.
ament_auto_add_executable(${PROJECT_NAME}_node_exe
  ${${PROJECT_NAME}_SRC}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  launch
)

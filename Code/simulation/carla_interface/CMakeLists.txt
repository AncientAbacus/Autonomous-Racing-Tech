# Minimum required CMake version for this project.
cmake_minimum_required(VERSION 3.5)

# Project name and language specifications.
project(carla_interface)

# Default C++ Standard to C++17, if no standard is set.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler options for GNU and Clang compilers.
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
  src/carla_interface.cpp
)

# Create this project as an executable.
ament_auto_add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SRC}
)

# If building for testing, find necessary dependencies and linting tools.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package the project, install files, and generate package-specific configuration files.
# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
  scripts
)

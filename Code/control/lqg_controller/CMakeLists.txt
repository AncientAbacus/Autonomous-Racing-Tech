cmake_minimum_required(VERSION 3.8)
project(lqg_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
ament_auto_find_build_dependencies()

set(LQG_CONTROLLER_LIB_SRC
  src/lkf_calculator.cpp
  src/lqg_controller.cpp
  src/lqr_calculator.cpp
)

set(LQG_CONTROLLER_LIB_HEADERS
  include/lqg_controller/lkf_calculator.hpp
  include/lqg_controller/lqg_controller.hpp
  include/lqg_controller/lqr_calculator.hpp
  include/lqg_controller/dynamic_bicycle_model.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${LQG_CONTROLLER_LIB_SRC}
  ${LQG_CONTROLLER_LIB_HEADERS}
)

# testing
if(BUILD_TESTING)
  set(ament_cmake_copyright_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(TEST_SOURCES test/test_lqg_controller.cpp)
  set(TEST_LQR_CONTROLLER_EXE test_lqr_controller)
  ament_add_gtest(${TEST_LQR_CONTROLLER_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_LQR_CONTROLLER_EXE} ${PROJECT_NAME})
endif()

ament_auto_package()

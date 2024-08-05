cmake_minimum_required(VERSION 3.8)
project(state_observers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  Eigen3
  rclcpp
  rclcpp_lifecycle
#  std_msgs
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/state_observers/state_observer.cpp
  src/state_observers/luenberger.cpp
  src/state_observers/kalman_filter.cpp
  src/state_observers_param/state_observer_param.cpp
  src/state_observers_param/luenberger_param.cpp
  src/state_observers_param/kalman_filter_param.cpp)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

install(DIRECTORY include/
  DESTINATION include/
)
install(TARGETS
  ${PROJECT_NAME}
  # EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
# ament_export_targets(
#   ${PROJECT_NAME}
#   HAS_LIBRARY_TARGET
# )

ament_package()

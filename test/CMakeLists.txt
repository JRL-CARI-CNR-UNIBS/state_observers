ament_add_gtest(state_observers_test state_observer_test.cpp)
target_link_libraries(state_observers_test
  ${PROJECT_NAME}
)
ament_target_dependencies(state_observers_test ${dependencies})

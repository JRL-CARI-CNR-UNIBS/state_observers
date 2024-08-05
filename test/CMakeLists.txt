ament_add_gtest(state_observers_test state_observer_test.cpp)
target_link_libraries(state_observers_test
  ${PROJECT_NAME}
)
ament_target_dependencies(state_observers_test ${dependencies})

ament_add_gtest(kalman_filter_test kalman_filter_test.cpp)
target_link_libraries(kalman_filter_test
  ${PROJECT_NAME}
)
ament_target_dependencies(kalman_filter_test ${dependencies})

ament_add_gtest(state_observers_param state_observer_param_test.cpp)
target_link_libraries(state_observers_param
  ${PROJECT_NAME}
)
ament_target_dependencies(state_observers_param ${dependencies})

ament_add_gtest(luenberger_param luenberger_param_test.cpp)
target_link_libraries(luenberger_param
  ${PROJECT_NAME}
)
ament_target_dependencies(luenberger_param ${dependencies})

ament_add_gtest(kalman_filter_param kalman_filter_param_test.cpp)
target_link_libraries(kalman_filter_param
  ${PROJECT_NAME}
)
ament_target_dependencies(kalman_filter_param ${dependencies})
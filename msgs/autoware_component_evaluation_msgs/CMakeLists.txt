cmake_minimum_required(VERSION 3.16)
project(autoware_component_evaluation_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ComponentReport.msg
  msg/EvaluationReport.msg
  msg/InterfaceObservation.msg
  msg/Verdict.msg
  DEPENDENCIES builtin_interfaces autoware_internal_metric_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

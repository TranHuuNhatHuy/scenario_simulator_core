cmake_minimum_required(VERSION 3.14)
project(autoware_core_message_bridge)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(message_bridge_node SHARED
  src/message_bridge_node.cpp
)
rclcpp_components_register_node(message_bridge_node
  PLUGIN "autoware_core_message_bridge::MessageBridgeNode"
  EXECUTABLE message_bridge_node_exe
)

ament_auto_package()

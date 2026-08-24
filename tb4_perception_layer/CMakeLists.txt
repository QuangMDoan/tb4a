cmake_minimum_required(VERSION 3.8)
project(tb4_perception_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── Message generation ────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticObstacle.msg"
  "msg/SemanticObstacleArray.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# ── Costmap plugin shared library ─────────────────────────────
add_library(perception_layer SHARED
  src/perception_layer.cpp
)

target_include_directories(perception_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(perception_layer PUBLIC cxx_std_17)

ament_target_dependencies(perception_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
)

# Link the generated message typesupport into the plugin library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(perception_layer "${cpp_typesupport_target}")

# ── BT condition node shared library ──────────────────────────
add_library(is_sign_nearby_bt_node SHARED
  src/is_sign_nearby.cpp
)

target_include_directories(is_sign_nearby_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(is_sign_nearby_bt_node PUBLIC cxx_std_17)

# Ensure BT_RegisterNodesFromPlugin symbol is exported
set_target_properties(is_sign_nearby_bt_node PROPERTIES
  CXX_VISIBILITY_PRESET default
  C_VISIBILITY_PRESET default
)

target_compile_definitions(is_sign_nearby_bt_node PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(is_sign_nearby_bt_node
  rclcpp
  behaviortree_cpp
  tf2_ros
)

target_link_libraries(is_sign_nearby_bt_node "${cpp_typesupport_target}")

# ── Plugin registration ───────────────────────────────────────
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins/perception_layer_plugin.xml)

# ── Install ───────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS perception_layer is_sign_nearby_bt_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES plugins/perception_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}/plugins
)

# ── Testing ───────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ── Export ────────────────────────────────────────────────────
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(perception_layer is_sign_nearby_bt_node)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  pluginlib
  std_msgs
  behaviortree_cpp
  tf2
  tf2_ros
  geometry_msgs
)

ament_package()

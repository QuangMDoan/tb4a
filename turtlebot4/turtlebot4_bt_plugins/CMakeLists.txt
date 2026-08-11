cmake_minimum_required(VERSION 3.8)
project(turtlebot4_bt_plugins)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/is_goal_behind_condition.cpp
    src/rotate_angle_action.cpp
    src/bt_plugins.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp 
    nav2_behavior_tree
    behaviortree_cpp
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    irobot_create_msgs
    pluginlib
)

pluginlib_export_plugin_description_file(nav2_behavior_tree plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib 
    LIBRARY DESTINATION lib 
    RUNTIME DESTINATION bin 
)

install(DIRECTORY include/
    DESTINATION include 
)

install(FILES plugin_description.xml
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
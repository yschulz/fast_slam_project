cmake_minimum_required(VERSION 3.8)
project(fast_slam_ros_visualization)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)


find_package(pluginlib REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(fast_slam_ros_msgs REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

set(fast_slam_ros_visualization_headers_to_moc
  include/fast_slam_ros_visualization/point_array_display.hpp
  include/fast_slam_ros_visualization/line_landmark_display.hpp
)

foreach(header "${fast_slam_ros_visualization_headers_to_moc}")
  qt5_wrap_cpp(fast_slam_ros_visualization_moc_files "${header}")
endforeach()

set(fast_slam_ros_visualization_source_files
  src/point_array_display.cpp
  src/line_landmark_display.cpp
)

add_library(fast_slam_ros_visualization SHARED
  ${fast_slam_ros_visualization_moc_files}
  ${fast_slam_ros_visualization_source_files}
)

target_include_directories(fast_slam_ros_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(fast_slam_ros_visualization PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

ament_target_dependencies(fast_slam_ros_visualization
  PUBLIC
  geometry_msgs
  fast_slam_ros_msgs
  rviz_common
  rviz_rendering
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(fast_slam_ros_visualization PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(fast_slam_ros_visualization HAS_LIBRARY_TARGET)

ament_export_dependencies(
  geometry_msgs
  fast_slam_ros_msgs
  rviz_common
  rviz_rendering
)

install(
  TARGETS fast_slam_ros_visualization
  EXPORT fast_slam_ros_visualization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
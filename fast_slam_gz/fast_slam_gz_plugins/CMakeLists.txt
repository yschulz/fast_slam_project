cmake_minimum_required(VERSION 3.8)
project(fast_slam_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fast_slam_ros_msgs REQUIRED)


add_library(fake_landmark_points SHARED src/fake_landmark_points.cpp)
target_include_directories(fake_landmark_points PRIVATE 
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(fake_landmark_points PRIVATE 
    gz-sim8::gz-sim8 
    gz-plugin2::register 
    ${rclcpp_TARGETS}
    ${geometry_msgs_TARGETS}
    ${fast_slam_ros_msgs_TARGETS}
)

install(TARGETS
fake_landmark_points
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_library(fake_landmark_poses SHARED src/fake_landmark_poses.cpp)
target_include_directories(fake_landmark_poses PRIVATE 
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(fake_landmark_poses PRIVATE 
    gz-sim8::gz-sim8 
    gz-plugin2::register 
    ${rclcpp_TARGETS}
    ${geometry_msgs_TARGETS}
)

install(TARGETS
fake_landmark_poses
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()

cmake_minimum_required(VERSION 3.8)
project(fast_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(libnabo REQUIRED)
find_package(OpenMP REQUIRED)


set(srcs 
  "src/Measurement.cpp"
  "src/MeasurementSet.cpp"
  "src/Particle.cpp"
  "src/ParticleSet.cpp"
  "src/PathList.cpp"
  "src/MapTree.cpp"
  "src/MapTreeIterator.cpp"
  "src/Helper.cpp"
)

add_library(${PROJECT_NAME} 
    ${srcs}
)

target_include_directories(${PROJECT_NAME}
               PUBLIC 
              "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
               "$<INSTALL_INTERFACE:include>"
               ${libnabo_INCLUDE_DIRS}
               ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  Eigen3
  libnabo
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  OpenMP::OpenMP_CXX
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(${PROJECT_NAME})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(
  Eigen3
  libnabo
  OpenMP
)

install(TARGETS
    ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)


if(BUILD_TESTING)
  include_directories(
    ${PROJECT_BINARY_DIR}/include
  )

  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()

  ament_add_gtest(map_tree_test test/map_tree_test.cpp)
  target_link_libraries(map_tree_test ${PROJECT_NAME} Eigen3::Eigen)
  target_include_directories(map_tree_test PUBLIC
    "include"
  )

  ament_add_gtest(path_test test/path_test.cpp)
  target_link_libraries(path_test ${PROJECT_NAME} Eigen3::Eigen)
  target_include_directories(path_test PUBLIC
    "include"
  )

  ament_add_gtest(measurement_test test/measurement_test.cpp)
  target_link_libraries(measurement_test ${PROJECT_NAME} Eigen3::Eigen)
  target_include_directories(measurement_test PUBLIC
    "include"
  )

  ament_add_gtest(particle_test test/particle_test.cpp)
  target_link_libraries(particle_test ${PROJECT_NAME} Eigen3::Eigen)
  target_include_directories(particle_test PUBLIC
    "include"
  )

  ament_add_gtest(map_tree_iterator_test test/map_tree_iterator_test.cpp)
  target_link_libraries(map_tree_iterator_test ${PROJECT_NAME} Eigen3::Eigen)
  target_include_directories(map_tree_iterator_test PUBLIC
    "include"
  )

endif()

ament_package()
